#include <memory>
#include <functional>
#include <vector>
#include <algorithm>
#include <cmath>

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
    // Subscriber: Subscribes to 3D obstacle bounding boxes from Perception (/perception/obstacles_only)
    obstacle_subscriber_ =
      this->create_subscription<vision_msgs::msg::Detection3DArray>(
        "/perception/obstacles_only",
        10,
        std::bind(
          &CostmapBridgeNode::obstacle_callback,
          this,
          std::placeholders::_1));

    // Publisher: Publishes point cloud for Nav2 native ObstacleLayer
    pointcloud_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/bridge/pointcloud",
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
    const float step = 0.05f;      // 5cm grid resolution matching costmap resolution
    const float epsilon = 1e-4f;

    for (const auto & detection : msg->detections)
    {
      const auto & center = detection.bbox.center.position;
      const auto & size = detection.bbox.size;
      const auto & orient = detection.bbox.center.orientation;

      // Ignore detections with invalid / non-positive footprint dimensions
      if (size.x <= 0.0f || size.y <= 0.0f)
      {
        continue;
      }

      // Compute yaw from quaternion: atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
      double siny_cosp = 2.0 * (orient.w * orient.z + orient.x * orient.y);
      double cosy_cosp = 1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z);
      double yaw = std::atan2(siny_cosp, cosy_cosp);

      bool has_rotation = (std::abs(yaw) > 1e-3);
      float cos_yaw = static_cast<float>(std::cos(yaw));
      float sin_yaw = static_cast<float>(std::sin(yaw));

      float half_x = static_cast<float>(size.x / 2.0);
      float half_y = static_cast<float>(size.y / 2.0);

      // Sample 3D bounding box footprint at 5cm grid resolution
      for (float dx = -half_x; dx <= half_x + epsilon; dx += step)
      {
        for (float dy = -half_y; dy <= half_y + epsilon; dy += step)
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

    if (sampled_points.empty())
    {
      return;
    }

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
