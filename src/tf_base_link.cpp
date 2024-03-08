#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class TFPointCloudTransformer : public rclcpp::Node
{
public:
  TFPointCloudTransformer()
  : Node("tf_base_link")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
      "point_cloud", 10,
      std::bind(&TFPointCloudTransformer::transformPointCloud, this, std::placeholders::_1));
    publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud>("transformed_point_cloud", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  void transformPointCloud(const sensor_msgs::msg::PointCloud::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud transformed_cloud;
    transformed_cloud.header = msg->header;

    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped =
        tf_buffer_->lookupTransform("base_link", msg->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", ex.what());
      return;
    }

    for (const auto & point : msg->points) {
      geometry_msgs::msg::PointStamped point_in, point_out;
      point_in.header = msg->header;
      point_in.point.x = point.x;
      point_in.point.y = point.y;
      point_in.point.z = point.z;

      try {
        tf2::doTransform(point_in, point_out, transformStamped);
      } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform point failed: %s", ex.what());
        continue;
      }

      geometry_msgs::msg::Point32 transformed_point;
      transformed_point.x = point_out.point.x;
      transformed_point.y = point_out.point.y;
      transformed_point.z = point_out.point.z;

      transformed_cloud.points.push_back(transformed_point);
    }

    publisher_->publish(transformed_cloud);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFPointCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}
