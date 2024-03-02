#include <cmath>
#include <queue>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class ObstacleClusterDetector : public rclcpp::Node
{
public:
  ObstacleClusterDetector() : Node("multi_obstacle_detector_pub")
  {
    marker_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", 10);

    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10, std::bind(&ObstacleClusterDetector::map_callback, this, std::placeholders::_1));
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
  {
    // 障害物セルのクラスタリング
    std::vector<std::vector<std::pair<int, int>>> clusters = cluster_obstacles(map);

    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    // 各クラスタの中心座標と大きさを計算してpublish
    for (auto & cluster : clusters) {
      double sum_x = 0, sum_y = 0;
      double max_distance = 0;
      for (auto & cell : cluster) {
        sum_x += cell.first;
        sum_y += cell.second;
      }
      double center_x = sum_x / cluster.size();
      double center_y = sum_y / cluster.size();

      for (auto & cell : cluster) {
        double dx = cell.first - center_x;
        double dy = cell.second - center_y;
        double distance = sqrt(dx * dx + dy * dy);
        if (distance > max_distance) {
          max_distance = distance;
        }
      }
      double obstacle_size = max_distance * map->info.resolution * 2;

      // Create a marker for this cluster
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = rclcpp::Node::now();
      marker.ns = "obstacle_clusters";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = center_x * map->info.resolution + map->info.origin.position.x;
      marker.pose.position.y = center_y * map->info.resolution + map->info.origin.position.y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = obstacle_size;
      marker.scale.y = obstacle_size;
      marker.scale.z = 0.1;   // Height of the cylinder
      marker.color.a = 0.75;  // 透明度
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;

      marker_array.markers.push_back(marker);
    }

    marker_publisher_->publish(marker_array);
  }

  std::vector<std::vector<std::pair<int, int>>> cluster_obstacles(
    const nav_msgs::msg::OccupancyGrid::SharedPtr map)
  {
    std::vector<std::vector<std::pair<int, int>>> clusters;
    std::vector<std::vector<bool>> visited(
      map->info.height, std::vector<bool>(map->info.width, false));
    int dx[4] = {1, -1, 0, 0};
    int dy[4] = {0, 0, 1, -1};
    int threshold = 50;  // 障害物とみなす閾値

    for (int y = 0; y < map->info.height; ++y) {
      for (int x = 0; x < map->info.width; ++x) {
        if (!visited[y][x] && map->data[x + y * map->info.width] > threshold) {
          // 新しいクラスタの開始
          std::vector<std::pair<int, int>> cluster;
          std::queue<std::pair<int, int>> q;
          q.push({x, y});
          visited[y][x] = true;

          while (!q.empty()) {
            auto cell = q.front();
            q.pop();
            cluster.push_back(cell);

            for (int i = 0; i < 4; ++i) {
              int nx = cell.first + dx[i], ny = cell.second + dy[i];

              // マップの範囲内かつ未訪問で閾値以上のセルをクラスタに追加
              if (
                nx >= 0 && nx < map->info.width && ny >= 0 && ny < map->info.height &&
                !visited[ny][nx] && map->data[nx + ny * map->info.width] > threshold) {
                q.push({nx, ny});
                visited[ny][nx] = true;
              }
            }
          }
          clusters.push_back(cluster);
        }
      }
    }
    return clusters;
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleClusterDetector>());
  rclcpp::shutdown();
  return 0;
}
