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
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

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
    declare_x_arg = DeclareLaunchArgument("x", default_value="0.38")
    declare_y_arg = DeclareLaunchArgument("y", default_value="-0.14")
    declare_z_arg = DeclareLaunchArgument("z", default_value="0.0")
    declare_roll_arg = DeclareLaunchArgument("roll", default_value="0.0")
    declare_pitch_arg = DeclareLaunchArgument("pitch", default_value="0.0")
    declare_yaw_arg = DeclareLaunchArgument("yaw", default_value="-1.51")

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

    hostage_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="hostage_gz_bridge",
        arguments=[
            "/hostage_1/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/hostage_1/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose",
            "/hostage_2/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/hostage_2/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose",
        ],
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(local_launch_dir / "office_spawn_robot.launch.py")),
        launch_arguments={
            "x": x,
            "y": y,
            "z": z,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }.items(),
    )

    focus_robot = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ign",
                    "service",
                    "-s",
                    "/gui/follow/offset",
                    "--reqtype",
                    "ignition.msgs.Vector3d",
                    "--reptype",
                    "ignition.msgs.Boolean",
                    "--timeout",
                    "3000",
                    "--req",
                    "x: -3.5 y: 0.0 z: 1.6",
                ],
                shell=False,
            ),
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
            declare_x_arg,
            declare_y_arg,
            declare_z_arg,
            declare_roll_arg,
            declare_pitch_arg,
            declare_yaw_arg,
            SetRemap("/diagnostics", "diagnostics"),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            gz_bridge,
            hostage_bridge,
            spawn_robot,
            focus_robot,
            rviz_launch,
        ]
    )
