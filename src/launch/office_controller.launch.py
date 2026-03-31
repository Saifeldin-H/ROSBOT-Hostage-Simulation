from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessIO
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString
from rosbot_utils.utils import find_device_port


def generate_launch_description() -> LaunchDescription:
    configuration = LaunchConfiguration("configuration")
    controller_config = LaunchConfiguration("controller_config")
    manipulator_serial_port = LaunchConfiguration("manipulator_serial_port")
    mecanum = LaunchConfiguration("mecanum")
    namespace = LaunchConfiguration("namespace")
    robot_model = LaunchConfiguration("robot_model")
    use_sim = LaunchConfiguration("use_sim", default="False")

    base_controller_prefix = PythonExpression(
        ["'mecanum_drive' if ", mecanum, " else 'diff_drive'"]
    )
    manipulator = PythonExpression(["'", configuration, "'.startswith('manipulation')"])
    manipulator_prefix = PythonExpression(["'manipulator_' if ", manipulator, " else ''"])
    controller_config_file = PythonExpression(
        ["'", base_controller_prefix, "' + '_' + '", manipulator_prefix, "' + 'controller.yaml'"]
    )
    default_controller_config = PathJoinSubstitution(
        [FindPackageShare("rosbot_controller"), "config", robot_model, controller_config_file]
    )

    declare_controller_config_arg = DeclareLaunchArgument(
        "controller_config",
        default_value=default_controller_config,
        description="Path to controller configuration file.",
    )
    declare_configuration_arg = DeclareLaunchArgument(
        "configuration",
        default_value="basic",
        choices=["basic", "telepresence", "autonomy", "manipulation", "manipulation_pro"],
    )
    default_manipulator_serial_port = find_device_port("0403", "6014", "/dev/ttyUSB0")
    declare_manipulator_serial_port_arg = DeclareLaunchArgument(
        "manipulator_serial_port",
        default_value=default_manipulator_serial_port,
        description="Port to connect to the manipulator.",
    )
    default_mecanum_value = PythonExpression(["'", robot_model, "' == 'rosbot_xl'"])
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value=default_mecanum_value,
        choices=["True", "False"],
    )
    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable("ROBOT_MODEL_NAME", default_value=""),
        choices=["rosbot", "rosbot_xl"],
    )

    ns = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    ns_controller_config = ReplaceString(controller_config, {"<namespace>/": ns})

    load_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rosbot_description"), "launch", "load_urdf.launch.py"]
            )
        ),
        launch_arguments={
            "configuration": configuration,
            "controller_config": ns_controller_config,
            "manipulator_serial_port": manipulator_serial_port,
            "mock_joints": "False",
            "robot_model": robot_model,
            "use_sim": use_sim,
        }.items(),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ns_controller_config],
        remappings=[
            ("drive_controller/cmd_vel_unstamped", "cmd_vel"),
            ("drive_controller/odom", "odometry/wheels"),
            ("drive_controller/transition_event", "_drive_controller/transition_event"),
            ("imu_sensor_node/imu", "/_imu/data_raw"),
            ("imu_broadcaster/transition_event", "_imu_broadcaster/transition_event"),
            (
                "joint_state_broadcaster/transition_event",
                "_joint_state_broadcaster/transition_event",
            ),
            ("~/motors_cmd", "/_motors_cmd"),
            ("~/motors_response", "/_motors_response"),
        ],
        condition=UnlessCondition(use_sim),
    )

    common_spawner_args = [
        "-c",
        "controller_manager",
        "--controller-manager-timeout",
        "60",
        "--switch-timeout",
        "60",
        "--service-call-timeout",
        "60",
    ]

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", *common_spawner_args],
    )
    imu_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster", *common_spawner_args],
    )
    drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_controller", *common_spawner_args],
    )

    manipulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rosbot_controller"), "launch", "manipulator.launch.py"]
            )
        ),
        condition=IfCondition(manipulator),
    )

    controllers = [joint_state_broadcaster, imu_broadcaster, drive_controller]

    delayed_controllers = TimerAction(period=12.0, actions=controllers)
    delayed_manipulator_launch = TimerAction(period=18.0, actions=[manipulator_launch])

    def check_if_log_is_fatal(event):
        red_color = "\033[91m"
        reset_color = "\033[0m"
        msg = event.text.decode().lower()
        if ("fatal" in msg or "failed" in msg) and "attempt" not in msg:
            print(f"{red_color}Fatal error: {event.text}. Emitting shutdown...{reset_color}")
            return EmitEvent(event=Shutdown(reason="Spawner failed"))

    controllers_monitor = GroupAction(
        [
            RegisterEventHandler(
                OnProcessIO(
                    target_action=spawner,
                    on_stderr=check_if_log_is_fatal,
                )
            )
            for spawner in controllers
        ]
    )

    return LaunchDescription(
        [
            declare_configuration_arg,
            declare_manipulator_serial_port_arg,
            declare_robot_model_arg,
            declare_mecanum_arg,
            declare_controller_config_arg,
            load_urdf,
            control_node,
            delayed_controllers,
            delayed_manipulator_launch,
            controllers_monitor,
        ]
    )
