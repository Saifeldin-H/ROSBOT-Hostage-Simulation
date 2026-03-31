from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def main() -> int:
    launch_file = Path(__file__).resolve().with_name("office_simulation.launch.py")

    description = LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_file)),
                launch_arguments={
                    "robot_model": "rosbot",
                    "gz_world": "/workspaces/project/src/husarion_gz_worlds/worlds/husarion_office.sdf",
                    "rviz": "False",
                }.items(),
            )
        ]
    )

    service = LaunchService()
    service.include_launch_description(description)
    return service.run()


if __name__ == "__main__":
    raise SystemExit(main())
