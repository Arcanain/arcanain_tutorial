#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class PiSubscriber : public rclcpp::Node
{
public:
  PiSubscriber()
  : Node("monte_carlo_pi_sub")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "monte_carlo_pi", 10, std::bind(&PiSubscriber::piCallback, this, std::placeholders::_1));
  }

private:
  void piCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received pi approximation: '%f'", msg->data);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PiSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
