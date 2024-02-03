// 必要なヘッダーファイルをインクルード
#include <chrono>      // 時間に関する標準ライブラリ
#include <functional>  // 関数オブジェクトに関する機能を提供
#include <memory>      // スマートポインタなどのメモリ管理機能を提供

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class Float32PubComponent : public rclcpp::Node
{
public:
  Float32PubComponent()
  : Node("float32_pub")
  {
    publisher_ = create_publisher<std_msgs::msg::Float32>("/data", 10);
    timer_ =
      create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&Float32PubComponent::publishData, this));
  }

private:
  void publishData()
  {
    std_msgs::msg::Float32 msg;
    msg.data = count_;

    RCLCPP_INFO(get_logger(), "Publishing: %f", msg.data);

    publisher_->publish(msg);
    count_++;

    if (count_ > count_limit_) {
      count_ = 0.0;
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  float count_ = 0;
  float count_limit_ = 40;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //auto node = std::make_shared<Float32PubComponent>();
  rclcpp::spin(std::make_shared<Float32PubComponent>());
  rclcpp::shutdown();
  return 0;
}