#include <memory>
#include <functional>
#include <vector>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "vision_msgs/msg/detection3_d_array.hpp"

class CostmapBridgeNode : public rclcpp::Node
{
public:
  CostmapBridgeNode()
  : Node("costmap_bridge_node")
  {
    declare_parameter("input_topic", "/perception/obstacles_only");
    declare_parameter("output_topic", "/bridge/pointcloud");
    declare_parameter("sample_spacing", 0.05);
    declare_parameter("max_points", 200000);
    if (!std::isfinite(get_parameter("sample_spacing").as_double()) ||
        get_parameter("sample_spacing").as_double() <= 0.0 || get_parameter("max_points").as_int() < 1) {
      throw std::invalid_argument("Positive sampling limits required");
    }
    // Subscriber: Subscribes to 3D obstacle bounding boxes from Perception (/perception/obstacles_only)
    obstacle_subscriber_ =
      this->create_subscription<vision_msgs::msg::Detection3DArray>(
        get_parameter("input_topic").as_string(),
        10,
        std::bind(
          &CostmapBridgeNode::obstacle_callback,
          this,
          std::placeholders::_1));

    // Publisher: Publishes point cloud for Nav2 native ObstacleLayer
    pointcloud_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
        get_parameter("output_topic").as_string(),
        10);

    RCLCPP_INFO(
      this->get_logger(),
      "Costmap Bridge Node started. Subscribing to /perception/obstacles_only (Detection3DArray) and sampling 3D footprints at 5cm into /bridge/pointcloud.");
  }

private:
  struct Point3D
  {
    float x;
    float y;
    float z;
  };

  void obstacle_callback(
    const vision_msgs::msg::Detection3DArray::SharedPtr msg)
  {
    std::vector<Point3D> sampled_points;
    const float step = static_cast<float>(get_parameter("sample_spacing").as_double());      // 5cm grid resolution matching costmap resolution
    const float epsilon = 1e-4f;

    const auto max_points = static_cast<std::size_t>(get_parameter("max_points").as_int());
    for (const auto & detection : msg->detections)
    {
      const auto & center = detection.bbox.center.position;
      const auto & size = detection.bbox.size;
      const auto & orient = detection.bbox.center.orientation;

      // Ignore detections with invalid / non-positive footprint dimensions
      if (!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(center.z) ||
          !std::isfinite(size.x) || !std::isfinite(size.y) || !std::isfinite(size.z) ||
          !std::isfinite(orient.x) || !std::isfinite(orient.y) || !std::isfinite(orient.z) || !std::isfinite(orient.w) ||
          size.x <= 0.0 || size.y <= 0.0 || size.z <= 0.0 || size.x > 100.0 || size.y > 100.0 ||
          sampled_points.size() >= max_points)
      {
        continue;
      }

      // Compute yaw from quaternion: atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
      const double norm2 = orient.x*orient.x + orient.y*orient.y + orient.z*orient.z + orient.w*orient.w;
      if (!std::isfinite(norm2) || norm2 < 1e-12) { continue; }
      double siny_cosp = 2.0 * (orient.w * orient.z + orient.x * orient.y) / norm2;
      double cosy_cosp = 1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z) / norm2;
      double yaw = std::atan2(siny_cosp, cosy_cosp);

      bool has_rotation = (std::abs(yaw) > 1e-3);
      float cos_yaw = static_cast<float>(std::cos(yaw));
      float sin_yaw = static_cast<float>(std::sin(yaw));

      float half_x = static_cast<float>(size.x / 2.0);
      float half_y = static_cast<float>(size.y / 2.0);
      if ((std::abs(orient.x) + std::abs(orient.y)) / std::sqrt(norm2) > 1e-3) {
        // Memory may express a tilted box in odom. Include vertical extent
        // in its conservative XY projection instead of silently dropping tilt.
        const double hx = size.x / 2.0, hy = size.y / 2.0, hz = size.z / 2.0;
        half_x = static_cast<float>(
          std::abs(1 - 2*(orient.y*orient.y + orient.z*orient.z)/norm2)*hx +
          std::abs(2*(orient.x*orient.y - orient.z*orient.w)/norm2)*hy +
          std::abs(2*(orient.x*orient.z + orient.y*orient.w)/norm2)*hz);
        half_y = static_cast<float>(
          std::abs(2*(orient.x*orient.y + orient.z*orient.w)/norm2)*hx +
          std::abs(1 - 2*(orient.x*orient.x + orient.z*orient.z)/norm2)*hy +
          std::abs(2*(orient.y*orient.z - orient.x*orient.w)/norm2)*hz);
        has_rotation = false;
      }

      // Sample 3D bounding box footprint at 5cm grid resolution
      for (float dx = -half_x; dx <= half_x + epsilon && sampled_points.size() < max_points; dx += step)
      {
        for (float dy = -half_y; dy <= half_y + epsilon && sampled_points.size() < max_points; dy += step)
        {
          float x, y;
          if (has_rotation)
          {
            x = static_cast<float>(center.x) + dx * cos_yaw - dy * sin_yaw;
            y = static_cast<float>(center.y) + dx * sin_yaw + dy * cos_yaw;
          }
          else
          {
            x = static_cast<float>(center.x) + dx;
            y = static_cast<float>(center.y) + dy;
          }

          sampled_points.push_back({
            x,
            y,
            static_cast<float>(center.z)
          });
        }
      }
    }

    // Empty is a valid observation update; do not retain a stale cloud.

    // Prepare PointCloud2 message
    sensor_msgs::msg::PointCloud2 pointcloud;
    pointcloud.header = msg->header;
    pointcloud.height = 1;
    pointcloud.width = static_cast<uint32_t>(sampled_points.size());
    pointcloud.is_dense = true;

    // Set XYZ fields and buffer size
    sensor_msgs::PointCloud2Modifier modifier(pointcloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(sampled_points.size());

    // Fill point cloud data
    sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud, "z");

    for (const auto & pt : sampled_points)
    {
      *iter_x = pt.x;
      *iter_y = pt.y;
      *iter_z = pt.z;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    // Publish to /bridge/pointcloud for Nav2 ObstacleLayer
    pointcloud_publisher_->publish(pointcloud);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Published PointCloud2 with %zu points covering %zu obstacles.",
      sampled_points.size(),
      msg->detections.size());
  }

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr
    obstacle_subscriber_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pointcloud_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CostmapBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
