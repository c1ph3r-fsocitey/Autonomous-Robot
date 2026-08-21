"""robot_state_publisher only.

Turns bot.urdf.xacro into the /robot_description parameter and publishes the
static parts of the TF tree. Everything else in this stack assumes this is
running, so it is included by bringup.launch.py rather than started by hand.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            FindPackageShare("bot_description"), "urdf", "bot.urdf.xacro"
        ]), " ",
        "use_mock_hardware:=", use_mock_hardware,
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Replace the ESP32 with mock_components/GenericSystem.",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[{
                "robot_description": ParameterValue(
                    robot_description_content, value_type=str),
                "use_sim_time": False,
                # The wheels are moved by joint_state_broadcaster, and every
                # other joint is fixed, so nothing here needs a GUI publisher.
                "publish_frequency": 30.0,
            }],
        ),
    ])
