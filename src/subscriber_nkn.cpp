#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

class Int32MultiArraySubscriber : public rclcpp::Node
{
public:
<<<<<<< HEAD:src/subscriber_nkn.cpp
  Int32MultiArraySubscriber()
  : Node("subscriber_nkn")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "topic", 10, std::bind(
        &Int32MultiArraySubscriber::topic_callback, this,
        std::placeholders::_1));
=======
  Int32MultiArraySubscriber() : Node("int32_array_sub")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "int32_array_data", 10,
      std::bind(&Int32MultiArraySubscriber::topic_callback, this, std::placeholders::_1));
>>>>>>> 865d87b (Temporary commit):src/int32_array_sub.cpp
  }

private:
  void topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const
  {
    // 受信した数値データをログに出力するために文字列に変換
    std::stringstream ss;
    for (const auto & value : msg->data) {
      ss << value << " ";
    }

    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", ss.str().c_str());
  }
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Int32MultiArraySubscriber>());
  rclcpp::shutdown();
  return 0;
}
