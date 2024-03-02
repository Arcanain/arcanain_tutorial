#include <cmath>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector()
  : Node("single_obstacle_detector_pub")
  {
    obstacle_position_publisher_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>("obstacle_position", 10);
    obstacle_size_publisher_ = this->create_publisher<std_msgs::msg::Float32>("obstacle_size", 10);
    marker_publisher_ =
      this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10, std::bind(&ObstacleDetector::map_callback, this, std::placeholders::_1));
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
  {
    int threshold = 50;  // 障害物とみなす閾値
    std::vector<std::pair<int, int>> obstacle_cells;

    // map.dataを走査して、閾値以上のセルを障害物として記録
    for (int y = 0; y < map->info.height; ++y) {
      for (int x = 0; x < map->info.width; ++x) {
        int index = x + y * map->info.width;
        if (map->data[index] > threshold) {
          obstacle_cells.push_back({x, y});
        }
      }
    }

    // 障害物セルの中心座標を計算（単純化のため平均を取る）
    double sum_x = 0, sum_y = 0;
    for (auto & cell : obstacle_cells) {
      sum_x += cell.first;
      sum_y += cell.second;
    }
    double center_x = sum_x / obstacle_cells.size();
    double center_y = sum_y / obstacle_cells.size();

    // 障害物の大きさを計算（単純化のため最大距離を利用）
    double max_distance = 0;
    for (auto & cell : obstacle_cells) {
      double dx = cell.first - center_x;
      double dy = cell.second - center_y;
      double distance = sqrt(dx * dx + dy * dy);
      if (distance > max_distance) {
        max_distance = distance;
      }
    }
    double obstacle_size = max_distance * map->info.resolution * 2;  // 往復のため2倍する

    // 障害物の位置をpublish
    auto obstacle_position_msg = geometry_msgs::msg::PointStamped();
    obstacle_position_msg.header.stamp = this->get_clock()->now();
    obstacle_position_msg.header.frame_id = "map";
    obstacle_position_msg.point.x = center_x * map->info.resolution + map->info.origin.position.x;
    obstacle_position_msg.point.y = center_y * map->info.resolution + map->info.origin.position.y;
    obstacle_position_msg.point.z = 0.0;  // 2Dの場合はZ座標は0
    obstacle_position_publisher_->publish(obstacle_position_msg);

    // 障害物のサイズをpublish
    auto obstacle_size_msg = std_msgs::msg::Float32();
    obstacle_size_msg.data = obstacle_size;
    obstacle_size_publisher_->publish(obstacle_size_msg);

    // 障害物のマーカーをパブリッシュ
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "obstacles";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = center_x * map->info.resolution + map->info.origin.position.x;
    marker.pose.position.y = center_y * map->info.resolution + map->info.origin.position.y;
    marker.pose.position.z = 0.0;  // 2Dマップの場合Z座標は0
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = obstacle_size;  // 直径
    marker.scale.y = obstacle_size;  // 直径
    marker.scale.z = 0.1;            // 高さは小さな値に
    marker.color.a = 0.5;            // 透明度
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_publisher_->publish(marker);

    RCLCPP_INFO(
      this->get_logger(), "Detected obstacle at (%.2f, %.2f) with size %.2f meters.",
      center_x * map->info.resolution + map->info.origin.position.x,
      center_y * map->info.resolution + map->info.origin.position.y, obstacle_size);
  }

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr obstacle_position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr obstacle_size_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}
