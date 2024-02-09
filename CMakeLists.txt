cmake_minimum_required(VERSION 3.8)
project(arcanain_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

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
ament_target_dependencies(odometry_pub rclcpp tf2_geometry_msgs nav_msgs geometry_msgs tf2_ros)

# laserscan_pub
add_executable(laserscan_pub src/laserscan_pub.cpp)
ament_target_dependencies(laserscan_pub rclcpp sensor_msgs tf2_ros geometry_msgs)

# twist_pub
add_executable(twist_pub src/twist_pub.cpp)
ament_target_dependencies(twist_pub rclcpp geometry_msgs)

add_executable(occupancy_grid_pub src/occupancy_grid_pub.cpp)
ament_target_dependencies(occupancy_grid_pub rclcpp nav_msgs)

# Install Cpp executables
install(TARGETS
  talker
  listener
  float32_pub
  odometry_pub
  laserscan_pub
  twist_pub
  occupancy_grid_pub
  DESTINATION lib/${PROJECT_NAME})

# Install other files
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

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
