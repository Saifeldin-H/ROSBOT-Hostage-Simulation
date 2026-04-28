from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="hostage_sim",
                executable="hostage_patrol_node.py",
                name="hostage_patrol",
                output="screen",
            ),
            Node(
                package="hostage_sim",
                executable="hostage_mission_node.py",
                name="hostage_mission",
                output="screen",
            ),
        ]
    )
