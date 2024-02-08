#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TwistPublisher : public rclcpp::Node
{
public:
  TwistPublisher()
  : Node("twist_pub")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&TwistPublisher::publish_twist, this));
  }

private:
  void publish_twist()
  {
    auto message = geometry_msgs::msg::Twist();
    // 線形速度の設定
    message.linear.x = 1.0;  // 前進速度 1 m/s
    message.linear.y = 0.0;
    message.linear.z = 0.0;

    // 角速度の設定
    message.angular.x = 0.0;
    message.angular.y = 0.0;
    message.angular.z = 0.5;  // 左回りの回転速度 0.5 rad/s

    RCLCPP_INFO(
      this->get_logger(), "Publishing: 'linear.x: '%.2f', angular.z: '%.2f'", message.linear.x,
      message.angular.z);
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistPublisher>());
  rclcpp::shutdown();
  return 0;
}
