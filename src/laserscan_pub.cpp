#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

class LaserScanPublisher : public rclcpp::Node
{
public:
  LaserScanPublisher()
  : Node("laserscan_pub")
  {
    scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 50);
    timer_ = this->create_wall_timer(1s, std::bind(&LaserScanPublisher::publish_scan, this));
    // 静的な変換を送信するタイマー
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    send_static_transform();
  }

private:
  void publish_scan()
  {
    auto scan = sensor_msgs::msg::LaserScan();
    scan.header.stamp = this->get_clock()->now();
    scan.header.frame_id = "laser_frame";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 3.14 / num_readings;
    scan.time_increment = (1 / laser_frequency) / num_readings;
    scan.range_min = 0.0;
    scan.range_max = 100.0;

    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);

    // 範囲と強度に一定の値を設定
    for (unsigned int i = 0; i < num_readings; ++i) {
      scan.ranges[i] = 5.0;         // 一定の範囲値 (例: 10メートル)
      scan.intensities[i] = 100.0;  // 一定の強度値
    }

    scan_pub->publish(scan);
  }

  void send_static_transform()
  {
    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = this->get_clock()->now();
    static_transform_stamped.header.frame_id = "base_link";
    static_transform_stamped.child_frame_id = "laser_frame";
    static_transform_stamped.transform.translation.x = 0.0;
    static_transform_stamped.transform.translation.y = 0.0;
    static_transform_stamped.transform.translation.z = 0.1;  // 仮にz方向に0.1mのオフセット
    static_transform_stamped.transform.rotation.x = 0.0;
    static_transform_stamped.transform.rotation.y = 0.0;
    static_transform_stamped.transform.rotation.z = 0.0;
    static_transform_stamped.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(static_transform_stamped);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  unsigned int num_readings = 100;
  double laser_frequency = 40.0;
  int count = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanPublisher>());
  rclcpp::shutdown();
  return 0;
}
