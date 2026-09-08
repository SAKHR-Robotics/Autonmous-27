#include <memory>
#include <functional>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "terrain_geometry_msgs/msg/obstacle_feature_array.hpp"


class CostmapBridgeNode : public rclcpp::Node
{
public:

  CostmapBridgeNode()
  : Node("costmap_bridge_node")
  {
    // Subscriber
    obstacle_subscriber_ =
      this->create_subscription<
        terrain_geometry_msgs::msg::ObstacleFeatureArray>(
          "/terrain/obstacle_features",
          10,
          std::bind(
            &CostmapBridgeNode::obstacle_callback,
            this,
            std::placeholders::_1));

    // Publisher
    pointcloud_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/bridge/pointcloud",
        10);

    RCLCPP_INFO(
      this->get_logger(),
      "Costmap Bridge Node started.");
  }


private:

  struct Point3D {
    float x;
    float y;
    float z;
  };

  void obstacle_callback(
    const terrain_geometry_msgs::msg::ObstacleFeatureArray::SharedPtr msg)
  {
    std::vector<Point3D> sampled_points;
    const float step = 0.05f; // 5 cm resolution matching costmap

    for (const auto & obstacle : msg->obstacles)
    {
      float min_x = std::min(static_cast<float>(obstacle.min_point.x), static_cast<float>(obstacle.max_point.x));
      float max_x = std::max(static_cast<float>(obstacle.min_point.x), static_cast<float>(obstacle.max_point.x));
      float min_y = std::min(static_cast<float>(obstacle.min_point.y), static_cast<float>(obstacle.max_point.y));
      float max_y = std::max(static_cast<float>(obstacle.min_point.y), static_cast<float>(obstacle.max_point.y));

      // If bounding box is degenerate, derive from width/depth or sensible defaults
      if (std::abs(max_x - min_x) < 0.01f || std::abs(max_y - min_y) < 0.01f)
      {
        float rx = (obstacle.depth > 0.05f) ? (obstacle.depth / 2.0f) : 0.35f;
        float ry = (obstacle.width > 0.05f) ? (obstacle.width / 2.0f) : 0.35f;
        min_x = static_cast<float>(obstacle.centroid.x) - rx;
        max_x = static_cast<float>(obstacle.centroid.x) + rx;
        min_y = static_cast<float>(obstacle.centroid.y) - ry;
        max_y = static_cast<float>(obstacle.centroid.y) + ry;
      }

      const float cz = (std::abs(obstacle.centroid.z) > 0.01) ? static_cast<float>(obstacle.centroid.z) : 0.2f;

      // Sample a 2D grid covering the entire footprint of the obstacle
      for (float x = min_x; x <= max_x; x += step)
      {
        for (float y = min_y; y <= max_y; y += step)
        {
          sampled_points.push_back({x, y, cz});
        }
      }

      // Also explicitly include centroid at base and elevated levels
      sampled_points.push_back({
        static_cast<float>(obstacle.centroid.x),
        static_cast<float>(obstacle.centroid.y),
        cz
      });
      sampled_points.push_back({
        static_cast<float>(obstacle.centroid.x),
        static_cast<float>(obstacle.centroid.y),
        cz + 0.1f
      });
    }

    if (sampled_points.empty())
    {
      return;
    }

    // Create PointCloud2
    sensor_msgs::msg::PointCloud2 pointcloud;
    pointcloud.header = msg->header;
    pointcloud.height = 1;
    pointcloud.width = static_cast<uint32_t>(sampled_points.size());

    sensor_msgs::PointCloud2Modifier modifier(pointcloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(sampled_points.size());

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

    pointcloud_publisher_->publish(pointcloud);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Published PointCloud2 with %zu sampled obstacle footprint points across %zu obstacles.",
      sampled_points.size(),
      msg->obstacles.size());
  }


  // Subscriber
  rclcpp::Subscription<
    terrain_geometry_msgs::msg::ObstacleFeatureArray>::SharedPtr
    obstacle_subscriber_;

  // Publisher
  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr
    pointcloud_publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<CostmapBridgeNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}