"""Sensors and the link to the ESP32.

  micro_ros_agent  -- turns the ESP32 into a ROS 2 participant. Nothing on the
                      wheel side works until this is running.
  rplidar_node     -- /scan
  imu_filter       -- /imu/data_raw -> /imu/data

Both serial devices are addressed through udev symlinks (/dev/esp32,
/dev/rplidar). Run bot_bringup/udev/install_udev.sh once before first use.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    esp32_port = LaunchConfiguration("esp32_port")
    esp32_baud = LaunchConfiguration("esp32_baud")
    use_micro_ros = LaunchConfiguration("use_micro_ros")
    use_lidar = LaunchConfiguration("use_lidar")

    rplidar_yaml = PathJoinSubstitution([
        FindPackageShare("bot_bringup"), "config", "rplidar.yaml"
    ])
    imu_filter_yaml = PathJoinSubstitution([
        FindPackageShare("bot_bringup"), "config", "imu_filter.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("esp32_port", default_value="/dev/esp32"),
        DeclareLaunchArgument("esp32_baud", default_value="115200"),
        DeclareLaunchArgument(
            "use_micro_ros", default_value="true",
            description="Set false when running with mock hardware."),
        DeclareLaunchArgument("use_lidar", default_value="true"),

        # --- ESP32 bridge ---------------------------------------------------
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent",
            arguments=["serial", "--dev", esp32_port, "-b", esp32_baud],
            condition=IfCondition(use_micro_ros),
            output="both",
        ),

        # --- RPLidar A1M8 ---------------------------------------------------
        Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            parameters=[rplidar_yaml],
            condition=IfCondition(use_lidar),
            output="both",
        ),

        # --- IMU orientation filter -----------------------------------------
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="imu_filter_madgwick",
            parameters=[imu_filter_yaml],
            output="both",
        ),
    ])
