#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"

using namespace std::chrono_literals;

class ColorRGBA_pub : public rclcpp::Node
{
public:
  ColorRGBA_pub()
  : Node("ColorRGBA_pub")
  {
    publisher_ = this->create_publisher<std_msgs::msg::ColorRGBA>("color_rgba", 10);

    color_msg_.r = 0.5;
    color_msg_.g = 0.3;
    color_msg_.b = 0.2;
    color_msg_.a = 1;

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), [this]() {publisher_->publish(color_msg_);});
  }

private:
  rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr publisher_;
  std_msgs::msg::ColorRGBA color_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorRGBA_pub>());
  rclcpp::shutdown();
  return 0;
}
