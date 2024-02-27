#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

using namespace std::chrono_literals;

class UInt8Publisher : public rclcpp::Node
{
public:
  UInt8Publisher()
  : Node("uint8_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::UInt8>("uint8_topic", 10);
    timer_ = this->create_wall_timer(
      1s, std::bind(&UInt8Publisher::publish_message, this));
  }

private:
  void publish_message()
  {
    auto message = std_msgs::msg::UInt8();
    message.data = 211;
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%u'", message.data);
  }

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UInt8Publisher>());
  rclcpp::shutdown();
  return 0;
}
