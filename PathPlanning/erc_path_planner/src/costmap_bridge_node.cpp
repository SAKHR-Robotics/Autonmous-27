#include <memory>
#include <functional>
#include <vector>
#include <algorithm>
#include <cmath>

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
    // Subscriber: يستمع لخصائص العوائق القادمة من Perception
    obstacle_subscriber_ =
      this->create_subscription<terrain_geometry_msgs::msg::ObstacleFeatureArray>(
        "/terrain/obstacle_features",
        10,
        std::bind(
          &CostmapBridgeNode::obstacle_callback,
          this,
          std::placeholders::_1));

    // Publisher: ينشر السحابة النقطية للـ Costmap
    pointcloud_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/bridge/pointcloud",
        10);

    RCLCPP_INFO(
      this->get_logger(),
      "Costmap Bridge Node started with 3D dense bounding volume sampling.");
  }

private:
  struct Point3D
  {
    float x;
    float y;
    float z;
  };

  void obstacle_callback(
    const terrain_geometry_msgs::msg::ObstacleFeatureArray::SharedPtr msg)
  {
    std::vector<Point3D> sampled_points;
    const double resolution = 0.05;  // دقة الخطوة 5cm مطابقة للـ costmap resolution
    const double epsilon = 1e-4;     // لضمان شمول الحد الأقصى max_point بدقة الفاصلة العائمة

    for (const auto & obstacle : msg->obstacles)
    {
      double min_x = std::min(obstacle.min_point.x, obstacle.max_point.x);
      double max_x = std::max(obstacle.min_point.x, obstacle.max_point.x);
      double min_y = std::min(obstacle.min_point.y, obstacle.max_point.y);
      double max_y = std::max(obstacle.min_point.y, obstacle.max_point.y);
      double min_z = std::min(obstacle.min_point.z, obstacle.max_point.z);
      double max_z = std::max(obstacle.min_point.z, obstacle.max_point.z);

      // حماية إضافية (Fallback): إذا لم تكن min/max محددة واستخدمت الأبعاد بدلاً منها
      if (std::abs(max_x - min_x) < 1e-5 && obstacle.depth > 0.0f)
      {
        min_x = obstacle.centroid.x - obstacle.depth / 2.0;
        max_x = obstacle.centroid.x + obstacle.depth / 2.0;
      }
      if (std::abs(max_y - min_y) < 1e-5 && obstacle.width > 0.0f)
      {
        min_y = obstacle.centroid.y - obstacle.width / 2.0;
        max_y = obstacle.centroid.y + obstacle.width / 2.0;
      }
      if (std::abs(max_z - min_z) < 1e-5 && obstacle.height > 0.0f)
      {
        min_z = obstacle.centroid.z - obstacle.height / 2.0;
        max_z = obstacle.centroid.z + obstacle.height / 2.0;
      }

      // أخذ عينات نقطية عبر كامل الصندوق المحيط ثلاثي الأبعاد (Full 3D Bounding Envelope)
      for (double x = min_x; x <= max_x + epsilon; x += resolution)
      {
        for (double y = min_y; y <= max_y + epsilon; y += resolution)
        {
          for (double z = min_z; z <= max_z + epsilon; z += resolution)
          {
            sampled_points.push_back({
              static_cast<float>(x),
              static_cast<float>(y),
              static_cast<float>(z)
            });
          }
        }
      }
    }

    if (sampled_points.empty())
    {
      return;
    }

    // تجهيز رسالة PointCloud2
    sensor_msgs::msg::PointCloud2 pointcloud;

    // الحفاظ على نفس الـ frame_id والـ timestamp للرسالة القادمة
    pointcloud.header = msg->header;
    if (pointcloud.header.stamp.sec == 0 && pointcloud.header.stamp.nanosec == 0) {
      pointcloud.header.stamp = this->now();
    }

    // سحابة نقطية غير مرتبة (1 row)
    pointcloud.height = 1;
    pointcloud.width = static_cast<uint32_t>(sampled_points.size());
    pointcloud.is_dense = true;

    // تحديد حقول XYZ وحجم الذاكرة
    sensor_msgs::PointCloud2Modifier modifier(pointcloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(sampled_points.size());

    // Iterators لتعبئة النقاط
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

    // نشر السحابة النقطية على التوبيك
    pointcloud_publisher_->publish(pointcloud);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Published PointCloud2 with %zu points covering %zu obstacles.",
      sampled_points.size(),
      msg->obstacles.size());
  }

  rclcpp::Subscription<terrain_geometry_msgs::msg::ObstacleFeatureArray>::SharedPtr
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
