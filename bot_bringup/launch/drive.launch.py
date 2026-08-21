"""Everything needed to drive the robot with the gamepad, in one command.

    ros2 launch bot_bringup drive.launch.py

That is the whole thing. Built for driving over SSH with no screen on the
robot: one terminal, one Ctrl-C to stop it all cleanly (the motors get a zero
command on the way out).

Controls:
    RT              forward
    LT              reverse
    left stick X    steer

Useful arguments:
    esp32_port:=/dev/esp32          if you installed the udev rules
    use_lidar:=true                 once the RPLidar is sorted
    use_twist_mux:=true             route through twist_mux instead of
                                    straight to the controller - needed if
                                    you want Nav2 running at the same time
    max_linear:=0.15                slower, for tight spaces
    deadman_button:=4               require LB to be held before it drives

What this does NOT start: SLAM, Nav2, RViz. Run those in their own terminals
so you can restart them without stopping the robot.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("bot_bringup")

    esp32_port = LaunchConfiguration("esp32_port")
    esp32_baud = LaunchConfiguration("esp32_baud")
    use_lidar = LaunchConfiguration("use_lidar")
    use_localization = LaunchConfiguration("use_localization")
    use_twist_mux = LaunchConfiguration("use_twist_mux")
    max_linear = LaunchConfiguration("max_linear")
    max_angular = LaunchConfiguration("max_angular")
    deadman_button = LaunchConfiguration("deadman_button")

    def include(name, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_share, "launch", name])
            ),
            **kwargs,
        )

    teleop_args = {
        "max_linear": max_linear,
        "max_angular": max_angular,
        "deadman_button": deadman_button,
    }

    return LaunchDescription([
        # ------------------------------------------------------------------
        # Arguments
        # ------------------------------------------------------------------
        DeclareLaunchArgument(
            "esp32_port", default_value="/dev/ttyACM0",
            description="Serial device for the ESP32. Use /dev/esp32 once the "
                        "udev rules are installed."),
        DeclareLaunchArgument("esp32_baud", default_value="115200"),
        DeclareLaunchArgument(
            "use_lidar", default_value="false",
            description="Start the RPLidar. Off by default so a lidar fault "
                        "cannot stop you driving."),
        DeclareLaunchArgument(
            "use_localization", default_value="true",
            description="Start the EKF, so /odometry/filtered and the "
                        "odom->base_footprint transform are available."),
        DeclareLaunchArgument(
            "use_twist_mux", default_value="false",
            description="false: teleop talks straight to diff_drive_controller "
                        "(fewer moving parts). true: route through twist_mux, "
                        "which is what you want if Nav2 is also running."),
        DeclareLaunchArgument("max_linear", default_value="0.22"),
        DeclareLaunchArgument("max_angular", default_value="1.50"),
        DeclareLaunchArgument(
            "deadman_button", default_value="-1",
            description="Button index that must be held to drive. -1 = none."),

        # ------------------------------------------------------------------
        # 1. The robot: TF, micro-ROS agent, ros2_control, EKF
        # ------------------------------------------------------------------
        include("bringup.launch.py", launch_arguments={
            "esp32_port": esp32_port,
            "esp32_baud": esp32_baud,
            "use_lidar": use_lidar,
            "use_localization": use_localization,
            "use_twist_mux": use_twist_mux,
        }.items()),

        # ------------------------------------------------------------------
        # 2. Gamepad teleop
        #
        # Delayed until after bringup's own 3 s control-stack timer, so
        # diff_drive_controller exists before anything tries to command it.
        # Starting teleop first is harmless but fills the log with warnings
        # about a topic nobody is subscribed to, which reads like a fault.
        # ------------------------------------------------------------------
        TimerAction(period=8.0, actions=[
            # Direct to the controller.
            include("teleop_race.launch.py",
                    launch_arguments={
                        **teleop_args,
                        "cmd_topic": "/diff_drive_controller/cmd_vel_unstamped",
                    }.items(),
                    condition=UnlessCondition(use_twist_mux)),

            # Through twist_mux, which gives the joystick priority over Nav2.
            include("teleop_race.launch.py",
                    launch_arguments={
                        **teleop_args,
                        "cmd_topic": "/cmd_vel_joy",
                    }.items(),
                    condition=IfCondition(use_twist_mux)),

            LogInfo(msg=[
                "\n"
                "==================================================\n"
                "  READY TO DRIVE\n"
                "     RT            forward\n"
                "     LT            reverse\n"
                "     left stick    steer\n"
                "  Ctrl-C stops everything and zeroes the motors.\n"
                "=================================================="
            ]),
        ]),
    ])
