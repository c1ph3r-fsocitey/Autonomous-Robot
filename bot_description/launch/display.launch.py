"""View the robot model on its own, with no hardware and no controllers.

    ros2 launch bot_description display.launch.py

Sliders appear for the two wheel joints. Useful for checking that the URDF
looks right and that the lidar/IMU frames are where you think they are,
without waiting for the whole stack to come up.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            FindPackageShare("bot_description"), "urdf", "bot.urdf.xacro"
        ]), " ",
        "use_mock_hardware:=true",
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare("bot_description"), "rviz", "bot.rviz"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": ParameterValue(
                    robot_description_content, value_type=str),
            }],
            output="both",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="both",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="log",
        ),
    ])
