#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class TFBaseLink : public rclcpp::Node
{
public:
  TFBaseLink()
  : Node("tf_base_link")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
      "point_cloud", 10, std::bind(&TFBaseLink::pointCloudCallback, this, std::placeholders::_1));
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg)
  {
    geometry_msgs::msg::PointStamped point_in, point_out;

    for (const auto & point : msg->points) {
      point_in.point.x = point.x;
      point_in.point.y = point.y;
      point_in.point.z = point.z;
      point_in.header.frame_id = msg->header.frame_id;
      point_in.header.stamp = rclcpp::Time(0);       // Use the latest available transform

      try {
        tf_buffer_->transform(point_in, point_out, "base_link");
        RCLCPP_INFO(
          this->get_logger(), "Transformed Point [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
          point_in.point.x, point_in.point.y, point_in.point.z,
          point_out.point.x, point_out.point.y, point_out.point.z);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform point: %s", ex.what());
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFBaseLink>());
  rclcpp::shutdown();
  return 0;
}
