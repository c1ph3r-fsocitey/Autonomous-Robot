"""robot_localization EKF.

Consumes /diff_drive_controller/odom and /imu/data, publishes
/odometry/filtered and the odom -> base_footprint transform.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf_yaml = PathJoinSubstitution([
        FindPackageShare("bot_bringup"), "config", "ekf.yaml"
    ])

    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            parameters=[ekf_yaml],
            output="both",
        ),
    ])
