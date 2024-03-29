cmake_minimum_required(VERSION 3.8)
project(arcanain_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)

# talker
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# listener
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# float32_pub
add_executable(float32_pub src/float32_pub.cpp)
ament_target_dependencies(float32_pub rclcpp std_msgs)

# odometry_pub
add_executable(odometry_pub src/odometry_pub.cpp)
ament_target_dependencies(odometry_pub rclcpp tf2_geometry_msgs nav_msgs geometry_msgs tf2_ros sensor_msgs)

# laserscan_pub
add_executable(laserscan_pub src/laserscan_pub.cpp)
ament_target_dependencies(laserscan_pub rclcpp sensor_msgs tf2_ros geometry_msgs)

# twist_pub
add_executable(twist_pub src/twist_pub.cpp)
ament_target_dependencies(twist_pub rclcpp geometry_msgs)

# uint8_pub
add_executable(uint8_pub src/uint8_pub.cpp)
ament_target_dependencies(uint8_pub rclcpp std_msgs)

# uint8_sub
add_executable(uint8_sub src/uint8_sub.cpp)
ament_target_dependencies(uint8_sub rclcpp std_msgs)

# occupancy_grid_pub
add_executable(occupancy_grid_pub src/occupancy_grid_pub.cpp)
ament_target_dependencies(occupancy_grid_pub rclcpp nav_msgs)

# imu_pub
add_executable(imu_pub src/imu_pub.cpp)
ament_target_dependencies(imu_pub rclcpp sensor_msgs tf2_ros geometry_msgs nav_msgs tf2_ros)

# point_cloud_pub
add_executable(point_cloud_pub src/point_cloud_pub.cpp)
ament_target_dependencies(point_cloud_pub rclcpp sensor_msgs geometry_msgs tf2_ros geometry_msgs)

# single_obstacle_detector_pub
add_executable(single_obstacle_detector_pub src/single_obstacle_detector_pub.cpp)
ament_target_dependencies(single_obstacle_detector_pub rclcpp nav_msgs geometry_msgs std_msgs visualization_msgs)

# multi_obstacle_detector_pub
add_executable(multi_obstacle_detector_pub src/multi_obstacle_detector_pub.cpp)
ament_target_dependencies(multi_obstacle_detector_pub rclcpp nav_msgs visualization_msgs)

# monte_carlo_pi_pub
add_executable(monte_carlo_pi_pub src/monte_carlo_pi_pub.cpp)
ament_target_dependencies(monte_carlo_pi_pub rclcpp std_msgs)

# monte_carlo_pi_sub
add_executable(monte_carlo_pi_sub src/monte_carlo_pi_sub.cpp)
ament_target_dependencies(monte_carlo_pi_sub rclcpp std_msgs)

# ColorRGBA_pub
add_executable(ColorRGBA_pub src/ColorRGBA_pub.cpp)
ament_target_dependencies(ColorRGBA_pub rclcpp std_msgs)

# ColorRGBA_sub
add_executable(ColorRGBA_sub src/ColorRGBA_sub.cpp)
ament_target_dependencies(ColorRGBA_sub rclcpp std_msgs)

# int32_array_pub
add_executable(int32_array_pub src/int32_array_pub.cpp)
ament_target_dependencies(int32_array_pub rclcpp std_msgs)

# int32_array_sub
add_executable(int32_array_sub src/int32_array_sub.cpp)
ament_target_dependencies(int32_array_sub rclcpp std_msgs)

# tf_baselink_sub
add_executable(tf_baselink src/tf_baselink.cpp)
ament_target_dependencies(tf_baselink rclcpp sensor_msgs tf2_ros tf2_geometry_msgs tf2)

# Install Cpp executables
install(TARGETS
  int32_array_pub
  int32_array_sub
  talker
  listener
  float32_pub
  odometry_pub
  laserscan_pub
  twist_pub
  uint8_pub
  uint8_sub
  occupancy_grid_pub
  imu_pub
  point_cloud_pub
  single_obstacle_detector_pub
  multi_obstacle_detector_pub
  ColorRGBA_pub
  ColorRGBA_sub
  monte_carlo_pi_pub
  monte_carlo_pi_sub
  tf_baselink
  DESTINATION lib/${PROJECT_NAME})

# Install other files
install(DIRECTORY
  launch
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  tf_baselink
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
