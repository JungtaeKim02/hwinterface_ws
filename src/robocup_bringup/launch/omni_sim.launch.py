from pathlib import Path
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def setup(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    bringup = Path(get_package_share_directory("robocup_bringup"))
    description = Path(get_package_share_directory("robocup_description"))
    gz_sim = Path(get_package_share_directory("ros_gz_sim"))
    controller_config = str(bringup / "config" / "sim_controllers.yaml")
    sim_robot = xacro.process_file(
        str(description / "urdf" / "robocup_base.urdf.xacro"),
        mappings={"controllers_file": controller_config},
    ).toxml()
    world = str(description / "worlds" / "robocup_test.sdf")
    gz_args = ["-r"]
    if argument("gui") == "false":
        gz_args.append("-s")
    gz_args.append(world)

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_sim / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": shlex.join(gz_args), "on_exit_shutdown": "true"}.items(),
    )
    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="gz_clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    robot_description = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": sim_robot, "use_sim_time": True}],
        output="screen",
    )
    spawn_robot = Node(
        package="ros_gz_sim", executable="create", name="spawn_robocup",
        arguments=["-world", "robocup_test", "-name", "robocup_omni",
                   "-topic", "/robot_description", "-z", "0.008"],
        output="screen",
    )
    spawn_sim_controllers = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "omni_drive", "--activate-as-group",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen",
    )

    def after_robot_spawn(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg="Gazebo failed to spawn robocup_omni")]
        return [spawn_sim_controllers]

    actions = [
        simulator, clock_bridge, robot_description,
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=after_robot_spawn)),
        spawn_robot,
    ]
    if argument("keyboard") == "true":
        actions.append(Node(
            package="robocup_bringup", executable="keyboard_teleop", name="keyboard_teleop",
            parameters=[{"use_sim_time": True, "uart_enabled": argument("uart") == "true"}],
            output="screen",
        ))
        if argument("joy") == "true":
            actions.append(LogInfo(msg="Keyboard control selected; Xbox input is disabled"))
    elif argument("joy") == "true":
        actions.extend([
            Node(
                package="joy", executable="game_controller_node", name="joy_node",
                parameters=[{"device_id": int(argument("joy_device_id")),
                             "deadzone": 0.08, "autorepeat_rate": 20.0, "use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="teleop_twist_joy", executable="teleop_node", name="teleop_twist_joy_node",
                parameters=[str(bringup / "config" / "xbox.yaml"), {"use_sim_time": True}],
                remappings=[("cmd_vel", "/omni_drive/cmd_vel")],
                output="screen",
            ),
        ])
    if argument("uart") == "true":
        uart_robot = xacro.process_file(
            str(bringup / "urdf" / "uart_system.urdf.xacro"),
            mappings={"serial_port": argument("serial_port"),
                      "can_preview_period_ms": argument("can_preview_period_ms")},
        ).toxml()
        uart_config = str(bringup / "config" / "uart_controllers.yaml")
        actions.extend([
            Node(
                package="robot_state_publisher", executable="robot_state_publisher",
                namespace="uart", name="description_publisher",
                parameters=[{"robot_description": uart_robot}],
                output="screen",
            ),
            Node(
                package="controller_manager", executable="ros2_control_node",
                namespace="uart", parameters=[uart_config],
                output="screen",
            ),
            Node(
                package="controller_manager", executable="spawner",
                name="spawn_uart_wheel_command_relay",
                arguments=["wheel_command_relay", "--controller-manager", "/uart/controller_manager",
                           "--controller-manager-timeout", "30", "--param-file", uart_config],
                output="screen",
            ),
        ])
    return actions


def generate_launch_description():
    boolean = ["true", "false"]
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", choices=boolean),
        DeclareLaunchArgument("keyboard", default_value="false", choices=boolean),
        DeclareLaunchArgument("joy", default_value="true", choices=boolean),
        DeclareLaunchArgument("joy_device_id", default_value="0"),
        DeclareLaunchArgument("uart", default_value="false", choices=boolean),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("can_preview_period_ms", default_value="200"),
        OpaqueFunction(function=setup),
    ])
