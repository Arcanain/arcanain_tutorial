#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("uint8_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::UInt8>("uint8_topic", 10);

  auto message = std_msgs::msg::UInt8();
  message.data = 211;

  publisher->publish(message);

  rclcpp::shutdown();
  return 0;
}
