#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ABCPublisher : public rclcpp::Node
{
public:
  ABCPublisher()
  : Node("std_msgs")
  {
    publisher_ = this->create_publisher<std_msgs::msg::UInt8>("uint8_topic", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&ABCPublisher::publish_message, this));
  }

private:
  void publish_message()
  {
    auto message = std_msgs::msg::UInt8();
    message.data = 111;
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ABCPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
