// 必要なヘッダーファイルをインクルード
#include <memory>  // スマートポインタなどのメモリ管理機能を提供

// ROS2のコア機能と標準メッセージ型をインクルード
#include "rclcpp/rclcpp.hpp"        // ROS2の基本的なノード機能を提供
#include "std_msgs/msg/string.hpp"  // 文字列型のメッセージを定義

// std::placeholders::_1 を使用しやすくするためのusing宣言
using std::placeholders::_1;

// MinimalSubscriberクラスはrclcpp::Nodeクラスを継承している
class MinimalSubscriber : public rclcpp::Node
{
public:
  // コンストラクタ
  MinimalSubscriber()
  : Node("minimal_subscriber")  // Nodeクラスのコンストラクタを呼び出し、ノード名を指定
  {
    // "topic"という名前のトピックにサブスクライブ。キューサイズは10。
    // std::bindを使ってコールバック関数topic_callbackを登録。
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  // トピックのコールバック関数
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    // 受け取ったメッセージの内容をログに出力
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  // サブスクリプションを保持するためのスマートポインタ
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

// main関数
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  // ROS2システムの初期化
  // MinimalSubscriberノードのインスタンスを作成し、スピンすることでコールバック関数を実行可能にする
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();  // ROS2システムのシャットダウン
  return 0;            // プログラム終了
}
