"""slam_toolbox.

    # build a new map
    ros2 launch bot_bringup slam.launch.py

    # localize against a serialized map you saved earlier
    ros2 launch bot_bringup slam.launch.py mode:=localization \\
        map_file:=/home/c1ph3r/maps/lab

Saving a map, once you are happy with it:

    # occupancy grid, for Nav2's map_server / AMCL
    ros2 run nav2_map_server map_saver_cli -f ~/maps/lab

    # pose graph, for slam_toolbox localization mode
    ros2 service call /slam_toolbox/serialize_map \\
        slam_toolbox/srv/SerializePoseGraph "{filename: '/home/c1ph3r/maps/lab'}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def _launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    map_file = LaunchConfiguration("map_file").perform(context)

    share = get_package_share_directory("bot_bringup")

    if mode == "localization":
        params_file = os.path.join(share, "config", "slam_localization.yaml")
        overrides = {"map_file_name": map_file} if map_file else {}
        if not map_file:
            raise RuntimeError(
                "mode:=localization needs map_file:=<path without extension>, "
                "pointing at a serialized pose graph (.posegraph + .data)."
            )
    else:
        params_file = os.path.join(share, "config", "slam_mapping.yaml")
        overrides = {}

    return [
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[params_file, overrides] if overrides else [params_file],
            output="both",
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode", default_value="mapping",
            description="'mapping' to build a new map, 'localization' to use a saved one.",
        ),
        DeclareLaunchArgument(
            "map_file", default_value="",
            description="Serialized pose graph path WITHOUT extension (localization mode).",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
