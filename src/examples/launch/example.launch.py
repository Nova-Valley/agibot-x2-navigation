from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="examples",
                executable="set_mc_action",
                name="set_mc_action",
                output="screen",
            ),
            Node(
                package="examples",
                executable="get_mc_action",
                name="get_mc_action",
                output="screen",
            ),
        ]
    )
