from pathlib import Path
import os

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


SCENARIOS = {
    "scenario_1": {
        "world": "/workspaces/project/src/husarion_gz_worlds/scenarios/scenario_1.sdf",
        "spawn": ("0.38", "-0.14", "0.0", "0.0", "0.0", "-1.51"),
    },
    "scenario_2": {
        "world": "/workspaces/project/src/husarion_gz_worlds/scenarios/scenario_2.sdf",
        "spawn": ("0.38", "-0.14", "0.0", "0.0", "0.0", "-1.51"),
    },
    "scenario_3": {
        "world": "/workspaces/project/src/husarion_gz_worlds/scenarios/scenario_3.sdf",
        "spawn": ("0.38", "-0.14", "0.0", "0.0", "0.0", "-1.51"),
    },
    "scenario_4": {
        "world": "/workspaces/project/src/husarion_gz_worlds/scenarios/scenario_4.sdf",
        "spawn": ("0.38", "-0.14", "0.0", "0.0", "0.0", "-1.51"),
    },
}


def main() -> int:
    launch_file = Path(__file__).resolve().with_name("office_simulation.launch.py")
    scenario_name = os.environ.get("RESCUE_SCENARIO", "scenario_1")
    scenario = SCENARIOS.get(scenario_name)
    if scenario is None:
        valid = ", ".join(sorted(SCENARIOS))
        raise RuntimeError(f"Unknown RESCUE_SCENARIO={scenario_name!r}. Valid values: {valid}")

    spawn_x, spawn_y, spawn_z, spawn_roll, spawn_pitch, spawn_yaw = scenario["spawn"]

    description = LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_file)),
                launch_arguments={
                    "robot_model": "rosbot",
                    "gz_world": scenario["world"],
                    "rviz": "False",
                    "x": spawn_x,
                    "y": spawn_y,
                    "z": spawn_z,
                    "roll": spawn_roll,
                    "pitch": spawn_pitch,
                    "yaw": spawn_yaw,
                }.items(),
            )
        ]
    )

    service = LaunchService()
    service.include_launch_description(description)
    return service.run()


if __name__ == "__main__":
    raise SystemExit(main())
