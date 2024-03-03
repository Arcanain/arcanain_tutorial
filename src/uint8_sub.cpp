#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

class UInt8Subscriber : public rclcpp::Node
{
public:
  UInt8Subscriber()
  : Node("uint8_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::UInt8>(
      "uint8_topic", 10, std::bind(&UInt8Subscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::UInt8::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%u'", msg->data);
  }

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UInt8Subscriber>());
  rclcpp::shutdown();
  return 0;
}
