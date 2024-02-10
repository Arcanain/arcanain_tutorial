#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class OccupancyGridPublisher : public rclcpp::Node
{
public:
  OccupancyGridPublisher()
  : Node("occupancy_grid_pub")
  {
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    timer_ =
      this->create_wall_timer(
      500ms, std::bind(
        &OccupancyGridPublisher::publish_occupancy_grid,
        this));
  }

private:
  void publish_occupancy_grid()
  {
    auto map = nav_msgs::msg::OccupancyGrid();
    map.header.stamp = this->get_clock()->now();
    map.header.frame_id = "map";

    // 地図のメタデータ設定
    map.info.resolution = 0.1; // メートル/ピクセル
    map.info.width = 100; // 10m x 10mの地図
    map.info.height = 100;
    map.info.origin.position.x = 0.0;
    map.info.origin.position.y = 0.0;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;

    // 地図データの初期化（すべてを未知(-1)に設定）
    map.data = std::vector<int8_t>(map.info.width * map.info.height, -1);

    // 地図データに何らかの情報を埋める（ここでは簡単のためランダムに占有セルを設定）
    for (size_t i = 0; i < map.data.size(); ++i) {
      if (std::rand() % 10 < 3) {   // 約30%の確率で障害物を配置
        map.data[i] = 100;
      }
    }

    publisher_->publish(map);
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGridPublisher>());
  rclcpp::shutdown();
  return 0;
}
