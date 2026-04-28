import os
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch_ros.actions import Node


def scenario_actor_xy(scenario: str, actor_name: str) -> tuple[float, float]:
    scenario_path = (
        Path("/workspaces/project/src/husarion_gz_worlds/scenarios")
        / f"{scenario}.sdf"
    )
    fallback = (1.45, -9.25)
    if not scenario_path.exists():
        return fallback

    root = ET.parse(scenario_path).getroot()
    actor = root.find(f".//actor[@name='{actor_name}']")
    if actor is None:
        return fallback

    pose = actor.findtext("pose", default="").split()
    if len(pose) < 2:
        return fallback

    return float(pose[0]), float(pose[1])


def generate_launch_description() -> LaunchDescription:
    scenario = os.environ.get("RESCUE_SCENARIO", "scenario_1")
    nodes = [
        Node(
            package="hostage_sim",
            executable="hostage_mission_node.py",
            name="hostage_mission",
            output="screen",
        ),
    ]

    if scenario == "scenario_1":
        hostage_x, hostage_y = scenario_actor_xy(scenario, "hostage_1")
        nodes.append(
            Node(
                package="hostage_sim",
                executable="hostage_patrol_node.py",
                name="hostage_1_patrol",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": "/hostage_1/cmd_vel",
                        "pose_topic": "/hostage_1/pose",
                        "waypoints": [
                            hostage_x,
                            hostage_y,
                            hostage_x,
                            hostage_y + 0.5,
                        ],
                        "linear_speed": 0.18,
                        "angular_speed": 0.8,
                        "goal_tolerance": 0.12,
                        "heading_tolerance": 0.35,
                        "loop": True,
                    }
                ],
            )
        )

    return LaunchDescription(nodes)
