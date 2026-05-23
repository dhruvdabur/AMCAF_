from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="amcaf",
            executable="main",
            name="amcaf_node",
            output="screen",
        ),
    ])
