#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

struct Obstacle
{
  double x, y;    // 障害物の中心座標 (メートル)
  double radius;  // 障害物の半径 (メートル)
  double margin;  // マージン (メートル)
};

class OccupancyGridPublisher : public rclcpp::Node
{
public:
  OccupancyGridPublisher()
  : Node("occupancy_grid_pub")
  {
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&OccupancyGridPublisher::publish_occupancy_grid, this));

    // 障害物のデータをobstacles_に追加
    obstacles_.push_back({2.0, 2.0, 0.5, 0.2});    // 1つ目の障害物
    obstacles_.push_back({4.0, 4.0, 0.3, 0.1});    // 2つ目の障害物
    obstacles_.push_back({4.0, -4.0, 0.8, 0.1});   // 3つ目の障害物
    obstacles_.push_back({-4.0, 4.0, 0.1, 0.1});   // 4つ目の障害物
    obstacles_.push_back({-1.0, -1.0, 0.1, 0.1});  // 5つ目の障害物
  }

private:
  void publish_occupancy_grid()
  {
    auto map = nav_msgs::msg::OccupancyGrid();
    map.header.stamp = this->get_clock()->now();
    map.header.frame_id = "map";

    // 地図のメタデータ設定
    map.info.resolution = 0.1;  // メートル/ピクセル
    map.info.width = 200;       // 20m x 20mの地図
    map.info.height = 200;
    map.info.origin.position.x = -(map.info.height * map.info.resolution) / 2.0;
    map.info.origin.position.y = -(map.info.width * map.info.resolution) / 2.0;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;

    // 地図データの初期化（すべてを未知(-1)に設定）
    map.data = std::vector<int8_t>(map.info.width * map.info.height, -1);

    // 複数の障害物に対してコストを計算
    for (const auto & obstacle : obstacles_) {
      // 障害物の中心位置と半径
      double obstacle_x = obstacle.x;            // メートル
      double obstacle_y = obstacle.y;            // メートル
      double obstacle_radius = obstacle.radius;  // メートル
      double margin = obstacle.margin;           // メートル

      // 障害物の領域を決定
      int min_x = std::max(
        0, static_cast<int>(
          (obstacle_x - obstacle_radius - map.info.origin.position.x) / map.info.resolution));
      int max_x = std::min(
        static_cast<int>(map.info.width - 1),
        static_cast<int>(
          (obstacle_x + obstacle_radius - map.info.origin.position.x) / map.info.resolution));
      int min_y = std::max(
        0, static_cast<int>(
          (obstacle_y - obstacle_radius - map.info.origin.position.y) / map.info.resolution));
      int max_y = std::min(
        static_cast<int>(map.info.height - 1),
        static_cast<int>(
          (obstacle_y + obstacle_radius - map.info.origin.position.y) / map.info.resolution));

      // 障害物の領域にコスト100を割り当てる
      for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
          double dx = x * map.info.resolution + map.info.origin.position.x - obstacle_x;
          double dy = y * map.info.resolution + map.info.origin.position.y - obstacle_y;
          double distance = sqrt(dx * dx + dy * dy);

          if (distance <= obstacle_radius) {
            map.data[y * map.info.width + x] = 100;  // 障害物領域
          } else if (distance <= obstacle_radius + margin) {
            map.data[y * map.info.width + x] = 50;  // マージン領域
          }
        }
      }
    }

    publisher_->publish(map);
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Obstacle> obstacles_;  // 障害物のリスト
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGridPublisher>());
  rclcpp::shutdown();
  return 0;
}
