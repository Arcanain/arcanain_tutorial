#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class ImuPublisher : public rclcpp::Node
{
public:
  ImuPublisher() : Node("imu_pub")
  {
    timer_ = this->create_wall_timer(500ms, std::bind(&ImuPublisher::publish_imu_data, this));

    odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&ImuPublisher::odom_callback, this, std::placeholders::_1));

    imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);

    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    send_static_transform();
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_orientation = msg->pose.pose.orientation;
  }

  void publish_imu_data()
  {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation = odom_orientation;
    imu_publisher->publish(imu_msg);
  }

  void send_static_transform()
  {
    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = this->get_clock()->now();
    static_transform_stamped.header.frame_id = "base_link";
    static_transform_stamped.child_frame_id = "imu_link";
    static_transform_stamped.transform.translation.x = -0.5;
    static_transform_stamped.transform.translation.y = 0.0;
    static_transform_stamped.transform.translation.z = 0.0;
    static_transform_stamped.transform.rotation.x = 0.0;
    static_transform_stamped.transform.rotation.y = 0.0;
    static_transform_stamped.transform.rotation.z = 0.0;
    static_transform_stamped.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(static_transform_stamped);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  geometry_msgs::msg::Quaternion odom_orientation;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPublisher>());
  rclcpp::shutdown();
  return 0;
}
