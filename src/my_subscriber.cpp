#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

void callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received: %d", msg->data);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("uint8_subscriber");
  auto subscriber = node->create_subscription<std_msgs::msg::UInt8>("uint8_topic", 10, callback);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
