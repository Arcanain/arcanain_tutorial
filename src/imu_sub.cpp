#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"  // tf2::Quaternionを使用するために追加
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"


using namespace std::chrono_literals;

class ImuSubscriber : public rclcpp::Node
{
public:
  ImuSubscriber()
  : Node("Imu_sub")
  {
    timer_ = this->create_wall_timer(1s, std::bind(&ImuSubscriber::publish_scan, this));

    imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&ImuSubscriber::imu_callback, this, std::placeholders::_1));

    odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // 静的な変換を送信するタイマー
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    send_static_transform();
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // imuメッセージから速度を更新
    ImuOrientation = msg->orientation;
    ImuId = msg->header;
  }
  void publish_scan()
  {
    nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = ImuId.stamp;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = ImuId.frame_id;

    odom_trans.transform.translation.x = 0.0;
    odom_trans.transform.translation.y = -3.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = ImuOrientation;
    odom_broadcaster->sendTransform(odom_trans);
  }

  void send_static_transform()
  {
    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = ImuId.stamp;
    static_transform_stamped.header.frame_id = "odom";
    static_transform_stamped.child_frame_id = ImuId.frame_id;
    static_transform_stamped.transform.translation.x = 0;
    static_transform_stamped.transform.translation.y = -3.0;
    static_transform_stamped.transform.translation.z = 0.0;  // 仮にz方向に0.1mのオフセット
    static_transform_stamped.transform.rotation = ImuOrientation;
    static_broadcaster_->sendTransform(static_transform_stamped);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
  geometry_msgs::msg::Quaternion ImuOrientation;
  std_msgs::msg::Header ImuId;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuSubscriber>());
  rclcpp::shutdown();
  return 0;
}