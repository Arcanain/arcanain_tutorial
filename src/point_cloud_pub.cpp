#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

class PointCloudPublisher : public rclcpp::Node
{
public:
  PointCloudPublisher() : Node("point_cloud_pub")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("point_cloud", 10);
    timer_ =
      this->create_wall_timer(500ms, std::bind(&PointCloudPublisher::publishPointCloud, this));

    // 静的な変換を送信するタイマー
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    send_static_transform();
  }

private:
  void publishPointCloud()
  {
    auto message = sensor_msgs::msg::PointCloud();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "point_cloud_frame";

    // ランダムな点群データの生成
    for (int i = 0; i < 100; ++i) {
      geometry_msgs::msg::Point32 point;
      point.x = static_cast<float>(10.0 * rand() / RAND_MAX - 5.0);
      point.y = static_cast<float>(10.0 * rand() / RAND_MAX - 5.0);
      point.z = static_cast<float>(10.0 * rand() / RAND_MAX - 5.0);
      message.points.push_back(point);
    }

    // 点群データのPublish
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published point cloud with %zu points", message.points.size());
  }

  void send_static_transform()
  {
    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = this->get_clock()->now();
    static_transform_stamped.header.frame_id = "base_link";
    static_transform_stamped.child_frame_id = "point_cloud_frame";
    static_transform_stamped.transform.translation.x = 0.5;
    static_transform_stamped.transform.translation.y = 0.0;
    static_transform_stamped.transform.translation.z = 0.5;
    static_transform_stamped.transform.rotation.x = 0.0;
    static_transform_stamped.transform.rotation.y = 0.0;
    static_transform_stamped.transform.rotation.z = 0.0;
    static_transform_stamped.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(static_transform_stamped);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
