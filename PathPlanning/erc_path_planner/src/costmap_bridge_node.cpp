#include <memory>
#include <functional>

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

  void obstacle_callback(
    const terrain_geometry_msgs::msg::ObstacleFeatureArray::SharedPtr msg)
  {
    const std::size_t number_of_obstacles =
      msg->obstacles.size();

    // Create PointCloud2
    sensor_msgs::msg::PointCloud2 pointcloud;

    // Keep same frame as obstacle message
    pointcloud.header = msg->header;

    // One row
    pointcloud.height = 1;

    // Number of points = number of obstacles
    pointcloud.width =
      static_cast<uint32_t>(number_of_obstacles);

    // Define XYZ fields
    sensor_msgs::PointCloud2Modifier modifier(pointcloud);

    modifier.setPointCloud2FieldsByString(
      1,
      "xyz");

    // Allocate memory
    modifier.resize(number_of_obstacles);

    // Iterators
    sensor_msgs::PointCloud2Iterator<float> iter_x(
      pointcloud,
      "x");

    sensor_msgs::PointCloud2Iterator<float> iter_y(
      pointcloud,
      "y");

    sensor_msgs::PointCloud2Iterator<float> iter_z(
      pointcloud,
      "z");

    // Convert each obstacle centroid to a point
    for (const auto & obstacle : msg->obstacles)
    {
      *iter_x = static_cast<float>(
        obstacle.centroid.x);

      *iter_y = static_cast<float>(
        obstacle.centroid.y);

      *iter_z = static_cast<float>(
        obstacle.centroid.z);

      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    // Publish
    pointcloud_publisher_->publish(pointcloud);

    RCLCPP_INFO(
      this->get_logger(),
      "Published PointCloud2 with %zu obstacle points.",
      number_of_obstacles);
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