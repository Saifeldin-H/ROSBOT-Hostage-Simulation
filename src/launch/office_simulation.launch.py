from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    local_launch_dir = Path(__file__).resolve().parent

    rviz = LaunchConfiguration("rviz")
    gz_world = LaunchConfiguration("gz_world")

    declare_rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="False",
        choices=["True", "true", "False", "false"],
    )
    declare_gz_world_arg = DeclareLaunchArgument(
        "gz_world",
        default_value="/workspaces/project/src/husarion_gz_worlds/worlds/husarion_office.sdf",
        description="Absolute path to SDF world file.",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_gz_worlds"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_log_level": "1", "gz_world": gz_world}.items(),
    )

    gz_bridge_config = PathJoinSubstitution(
        [FindPackageShare("rosbot_gazebo"), "config", "gz_bridge.yaml"]
    )
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": gz_bridge_config}],
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(local_launch_dir / "office_spawn_robot.launch.py"))
    )

    focus_robot = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ign",
                    "service",
                    "-s",
                    "/gui/move_to",
                    "--reqtype",
                    "ignition.msgs.StringMsg",
                    "--reptype",
                    "ignition.msgs.Boolean",
                    "--timeout",
                    "3000",
                    "--req",
                    'data: "rosbot"',
                ],
                shell=False,
            ),
            ExecuteProcess(
                cmd=[
                    "ign",
                    "service",
                    "-s",
                    "/gui/follow",
                    "--reqtype",
                    "ignition.msgs.StringMsg",
                    "--reptype",
                    "ignition.msgs.Boolean",
                    "--timeout",
                    "3000",
                    "--req",
                    'data: "rosbot"',
                ],
                shell=False,
            ),
        ],
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rosbot_description"), "launch", "rviz.launch.py"]
            )
        ),
        launch_arguments={"namespace": ""}.items(),
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            declare_rviz_arg,
            declare_gz_world_arg,
            SetRemap("/diagnostics", "diagnostics"),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            gz_bridge,
            spawn_robot,
            focus_robot,
            rviz_launch,
        ]
    )
