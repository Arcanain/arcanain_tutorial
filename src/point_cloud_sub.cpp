#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

class PointCloudSubscriber : public rclcpp::Node
{
public:
  PointCloudSubscriber()
  : Node("point_cloud_sub")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
      "point_cloud", 10,
      std::bind(&PointCloudSubscriber::pointCloudCallback, this, std::placeholders::_1));
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", msg->points.size());
    for (const auto & point : msg->points) {
      RCLCPP_INFO(this->get_logger(), "Point: x=%f, y=%f, z=%f", point.x, point.y, point.z);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudSubscriber>());
  rclcpp::shutdown();
  return 0;
}
