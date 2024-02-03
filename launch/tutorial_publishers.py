from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'arcanain_tutorial'
    rviz_file_name = "arcanain_tutorial.rviz"

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

    nodes = [
        rviz_node,
        minimal_publisher_node,
        minimal_subscriber_node,
        float32_node,
    ]

    return LaunchDescription(nodes)
