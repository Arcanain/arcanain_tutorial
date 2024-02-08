#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"  // Pathを使用するために追加
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"  // tf2::Quaternionを使用するために追加
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class OdometryPublisher : public rclcpp::Node
{
public:
  OdometryPublisher()
  : Node("odometry_pub")
  {
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    path_pub = this->create_publisher<nav_msgs::msg::Path>("odom_path", 50);
    odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    x = 0.0;
    y = 0.0;
    th = 0.0;
    vx = 0.1;
    vy = -0.1;
    vth = 0.1;

    current_time = this->get_clock()->now();
    last_time = this->get_clock()->now();

    path.header.frame_id = "odom";  // パスのフレームIDを設定

    timer_ = this->create_wall_timer(100ms, std::bind(&OdometryPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    current_time = this->get_clock()->now();

    double dt = (current_time - last_time).seconds();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, th);  // ロール、ピッチ、ヨーをセット
    geometry_msgs::msg::Quaternion odom_quat_msg =
      tf2::toMsg(odom_quat);  // tf2::Quaternionからgeometry_msgs::msg::Quaternionに変換

    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat_msg;

    odom_broadcaster->sendTransform(odom_trans);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat_msg;

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    odom_pub->publish(odom);

    // パスに現在の位置を追加
    geometry_msgs::msg::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = x;
    this_pose_stamped.pose.position.y = y;
    this_pose_stamped.pose.orientation = odom_quat_msg;
    this_pose_stamped.header.stamp = current_time;
    this_pose_stamped.header.frame_id = "odom";
    path.poses.push_back(this_pose_stamped);

    // パスを公開
    path_pub->publish(path);

    last_time = current_time;
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;  // パスを公開するためのパブリッシャー
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path path;  // Pathメッセージのメンバ変数を追加
  double x, y, th, vx, vy, vth;
  rclcpp::Time current_time, last_time;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}
