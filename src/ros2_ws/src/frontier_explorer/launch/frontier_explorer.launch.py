from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value="/workspaces/project/src/config/frontier_params.yaml",
                description="Path to the frontier explorer parameter file.",
            ),
            Node(
                package="frontier_explorer",
                executable="frontier_explorer_node",
                name="frontier_explorer",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
