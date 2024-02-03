from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'arcanain_tutorial'

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
        minimal_publisher_node,
        minimal_subscriber_node,
        float32_node,
    ]

    return LaunchDescription(nodes)
