#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"

class ColorRGBA_sub : public rclcpp::Node
{
public:
  ColorRGBA_sub()
  : Node("ColorRGBA_sub")
  {
    subscription_ = this->create_subscription<std_msgs::msg::ColorRGBA>(
      "color_rgba", 10, std::bind(&ColorRGBA_sub::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::ColorRGBA::SharedPtr msg) const
  {
    RCLCPP_INFO(
      this->get_logger(), "I heard: %.2f, %.2f, %.2f, %.2f", msg->r, msg->g, msg->b, msg->a);
  }

  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorRGBA_sub>());
  rclcpp::shutdown();
  return 0;
}
