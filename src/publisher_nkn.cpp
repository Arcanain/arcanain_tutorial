#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

class Int32MultiArrayPublisher : public rclcpp::Node
{
public:
  Int32MultiArrayPublisher()
  : Node("publisher_nkn")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("topic", 10);
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&Int32MultiArrayPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Int32MultiArray();
    message.data = {1, 2, 3, 4, 5};
    std::stringstream ss;
    for (int value : message.data) {
      ss << value << " ";
    }
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", ss.str().c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Int32MultiArrayPublisher>());
  rclcpp::shutdown();
  return 0;
}
