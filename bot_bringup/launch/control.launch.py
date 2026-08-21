"""ros2_control stack: controller_manager, the two controllers, and twist_mux.

Start order matters here. joint_state_broadcaster has to be up before
diff_drive_controller, otherwise the controller manager briefly has a claimed
set of interfaces with nothing publishing joint states and RViz shows a
collapsed robot. The event handlers below enforce that.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_twist_mux = LaunchConfiguration("use_twist_mux")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            FindPackageShare("bot_description"), "urdf", "bot.urdf.xacro"
        ]), " ",
        "use_mock_hardware:=", use_mock_hardware,
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("bot_bringup"), "config", "controllers.yaml"
    ])
    twist_mux_yaml = PathJoinSubstitution([
        FindPackageShare("bot_bringup"), "config", "twist_mux.yaml"
    ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="both",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller",
                   "--controller-manager", "/controller_manager"],
    )

    # Everything that wants to drive the robot publishes a Twist somewhere;
    # twist_mux picks a winner and feeds diff_drive_controller.
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[twist_mux_yaml],
        remappings=[
            ("/cmd_vel_out", "/diff_drive_controller/cmd_vel_unstamped"),
        ],
        condition=IfCondition(use_twist_mux),
        output="both",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Replace the ESP32 with mock_components/GenericSystem.",
        ),
        DeclareLaunchArgument(
            "use_twist_mux",
            default_value="true",
            description="Start twist_mux (needs ros-humble-twist-mux). If "
                        "false, teleop and Nav2 must publish directly to "
                        "/diff_drive_controller/cmd_vel_unstamped.",
        ),

        control_node,
        twist_mux_node,
        jsb_spawner,

        RegisterEventHandler(
            OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[diff_drive_spawner],
            )
        ),
    ])
