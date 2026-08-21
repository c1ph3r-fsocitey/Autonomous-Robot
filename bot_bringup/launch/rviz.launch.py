"""RViz with a config that already has the robot model, /scan and the costmaps.

    # while mapping
    ros2 launch bot_bringup rviz.launch.py

    # while navigating
    ros2 launch bot_bringup rviz.launch.py config:=nav

Run this on a laptop on the same network rather than on the Pi if you can -
RViz will happily eat most of a Pi 4's CPU and starve the control loop.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config")

    rviz_config = PathJoinSubstitution([
        FindPackageShare("bot_description"), "rviz",
        [config, TextSubstitution(text=".rviz")],
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value="bot",
            description="RViz config name in bot_description/rviz: 'bot' or 'nav'.",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": False}],
            output="log",
        ),
    ])
