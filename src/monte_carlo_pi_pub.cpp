#include <chrono>
#include <cstdlib>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PiPublisher : public rclcpp::Node
{
public:
  PiPublisher() : Node("monte_carlo_pi_pub")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("monte_carlo_pi", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&PiPublisher::publishPiApproximation, this));
    std::srand(std::time(nullptr));  // 乱数の初期化
  }

private:
  void publishPiApproximation()
  {
    auto message = std_msgs::msg::Float64();
    message.data = calculatePi(10000);  // 10000回の試行でπを計算
    RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
    publisher_->publish(message);
  }

  double calculatePi(int trials)
  {
    int inside_circle = 0;
    for (int i = 0; i < trials; ++i) {
      double x = static_cast<double>(std::rand()) / RAND_MAX;
      double y = static_cast<double>(std::rand()) / RAND_MAX;
      if (x * x + y * y <= 1.0) {
        inside_circle++;
      }
    }
    return 4.0 * inside_circle / trials;
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PiPublisher>());
  rclcpp::shutdown();
  return 0;
}
