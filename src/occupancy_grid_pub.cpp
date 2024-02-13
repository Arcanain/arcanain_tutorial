#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class OccupancyGridPublisher : public rclcpp::Node
{
public:
  OccupancyGridPublisher()
  : Node("occupancy_grid_pub")
  {
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&OccupancyGridPublisher::publish_occupancy_grid, this));
  }

private:
  void publish_occupancy_grid()
  {
    auto occupancy_grid = nav_msgs::msg::OccupancyGrid();
    occupancy_grid.header.stamp = this->get_clock()->now();
    occupancy_grid.header.frame_id = "map";

    // グリッドマップのメタデータ設定
    occupancy_grid.info.resolution = 0.1;  // グリッドの解像度 [m/cell]
    occupancy_grid.info.width = 100;       // グリッドの幅 [cell]
    occupancy_grid.info.height = 100;      // グリッドの高さ [cell]
    occupancy_grid.info.origin.position.x = 0.0;
    occupancy_grid.info.origin.position.y = 0.0;
    occupancy_grid.info.origin.position.z = 0.0;
    occupancy_grid.info.origin.orientation.w = 1.0;

    // グリッドデータの設定 (例: 全てのセルを未知の状態(-1)に設定)
    occupancy_grid.data.resize(occupancy_grid.info.width * occupancy_grid.info.height, -1);

    publisher_->publish(occupancy_grid);
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
