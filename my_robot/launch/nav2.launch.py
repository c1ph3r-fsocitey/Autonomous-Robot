from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('my_robot')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': os.path.join(pkg, 'maps', 'my_map.yaml'),
                'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
                'use_sim_time': 'false'
            }.items()
        )
    ])
