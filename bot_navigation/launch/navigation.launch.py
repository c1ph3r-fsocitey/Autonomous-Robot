"""Nav2 core: planner, controller, behaviours, BT navigator, smoother.

This does NOT provide the map -> odom transform. Pair it with exactly one of:

    # A. mapping and navigating at the same time
    ros2 launch bot_bringup     slam.launch.py
    ros2 launch bot_navigation  navigation.launch.py

    # B. navigating a map you saved earlier
    ros2 launch bot_navigation  amcl.launch.py map:=/home/c1ph3r/maps/lab.yaml
    ros2 launch bot_navigation  navigation.launch.py

Assumes bot_bringup/bringup.launch.py is already running.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")

    default_params = PathJoinSubstitution([
        FindPackageShare("bot_navigation"), "config", "nav2_params.yaml"
    ])

    nav2_navigation_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="Full path to the Nav2 parameters file."),
        DeclareLaunchArgument(
            "autostart", default_value="true",
            description="Automatically bring the Nav2 lifecycle nodes up."),
        DeclareLaunchArgument(
            "log_level", default_value="info"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_navigation_launch),
            launch_arguments={
                "use_sim_time": "false",
                "params_file": params_file,
                "autostart": autostart,
                "use_composition": "False",
                "log_level": log_level,
            }.items(),
        ),
    ])
