"""map_server + AMCL: localize against a saved occupancy grid.

    ros2 launch bot_navigation amcl.launch.py map:=/home/c1ph3r/maps/lab.yaml

Then set the initial pose with RViz's "2D Pose Estimate" button, or:

    ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \\
      "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0},
        orientation: {w: 1.0}}}}"

Alternative: slam_toolbox in localization mode
(`ros2 launch bot_bringup slam.launch.py mode:=localization map_file:=...`).
That reuses the same scan matcher you mapped with and is usually more robust
on a small robot with a cheap lidar - at roughly twice the CPU cost.
Run one or the other, never both: they both publish map -> odom.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    default_params = PathJoinSubstitution([
        FindPackageShare("bot_navigation"), "config", "nav2_params.yaml"
    ])

    nav2_localization_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"), "launch", "localization_launch.py"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            description="Full path to the map YAML produced by map_saver_cli."),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("autostart", default_value="true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_localization_launch),
            launch_arguments={
                "map": map_yaml,
                "use_sim_time": "false",
                "params_file": params_file,
                "autostart": autostart,
                "use_composition": "False",
            }.items(),
        ),
    ])
