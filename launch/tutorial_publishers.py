import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'arcanain_tutorial'
    rviz_file_name = "arcanain_tutorial.rviz"

    file_path = os.path.expanduser('~/ros2_ws/src/arcanain_tutorial/urdf/sam_bot_description.urdf')

    with open(file_path, 'r') as file:
        robot_description = file.read()

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "rviz", rviz_file_name]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    robot_description_rviz_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_rviz_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='both',
        parameters=[{'joint_state_publisher': robot_description}]
    )

    minimal_publisher_node = Node(
        package=package_name,
        executable='talker',
        output="screen",
    )

    minimal_subscriber_node = Node(
        package=package_name,
        executable='listener',
        output="screen",
    )

    float32_node = Node(
        package=package_name,
        executable='float32_pub',
        output="screen",
    )

    odometry_node = Node(
        package=package_name,
        executable='odometry_pub',
        output="screen",
    )

    laserscan_node = Node(
        package=package_name,
        executable='laserscan_pub',
        output="screen",
    )

    twist_node = Node(
        package=package_name,
        executable='twist_pub',
        output="screen",
    )

    uint8_pub_node = Node(
        package=package_name,
        executable='uint8_pub',
        output="screen",
    )

    uint8_sub_node = Node(
        package=package_name,
        executable='uint8_sub',
    )

    occupancy_grid_node = Node(
        package=package_name,
        executable='occupancy_grid_pub',
        output="screen",
    )

    point_cloud_node = Node(
        package=package_name,
        executable='point_cloud_pub',
        output="screen",
    )

    single_obstacle_detector_node = Node(
        package=package_name,
        executable='single_obstacle_detector_pub',
        output="screen",
    )

    multi_obstacle_detector_node = Node(
        package=package_name,
        executable='multi_obstacle_detector_pub',
        output="screen",
    )

    ColorRGBA_pub_node = Node(
        package=package_name,
        executable='ColorRGBA_pub',
        output="screen",
    )

    ColorRGBA_sub_node = Node(
        package=package_name,
        executable='ColorRGBA_sub',
        output="screen",
    )

    imu_pub_node = Node(
        package=package_name,
        executable='imu_pub',
        output="screen",
    )

    int32_array_pub_node = Node(
        package=package_name,
        executable='int32_array_pub',
        output="screen",
    )

    int32_array_sub_node = Node(
        package=package_name,
        executable='int32_array_sub',

        output="screen",
    )

    nodes = [
        rviz_node,
        robot_description_rviz_node,
        joint_state_publisher_rviz_node,
        minimal_publisher_node,
        minimal_subscriber_node,
        float32_node,
        odometry_node,
        laserscan_node,
        twist_node,
        uint8_sub_node,
        uint8_pub_node,
        occupancy_grid_node,
        point_cloud_node,
        single_obstacle_detector_node,
        multi_obstacle_detector_node,
        ColorRGBA_pub_node,
        ColorRGBA_sub_node,
        imu_pub_node,
        int32_array_pub_node,
        int32_array_sub_node,
    ]

    return LaunchDescription(nodes)
