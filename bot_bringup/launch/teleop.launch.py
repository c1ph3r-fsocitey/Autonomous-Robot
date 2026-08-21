"""Joystick teleop.

    ros2 launch bot_bringup teleop.launch.py

Publishes to /cmd_vel_joy, which twist_mux gives the highest priority - so a
joystick input always beats Nav2.

For keyboard instead, run this in its own terminal (it needs the focus):

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -r /cmd_vel:=/cmd_vel_key
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_yaml = PathJoinSubstitution([
        FindPackageShare("bot_bringup"), "config", "joy_teleop.yaml"
    ])

    return LaunchDescription([
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[joy_yaml],
            output="both",
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            parameters=[joy_yaml],
            remappings=[("/cmd_vel", "/cmd_vel_joy")],
            output="both",
        ),
    ])
