"""Master bring-up: everything the robot needs to sit there and be drivable.

    ros2 launch bot_bringup bringup.launch.py

    # if the udev rules are not installed yet and the ESP32 is on ttyACM0:
    ros2 launch bot_bringup bringup.launch.py esp32_port:=/dev/ttyACM0

    # no ESP32 attached, for testing launch files / Nav2 / RViz on a desk:
    ros2 launch bot_bringup bringup.launch.py use_mock_hardware:=true \\
        use_micro_ros:=false use_lidar:=false

    # twist_mux not installed yet - teleop then has to publish straight to
    # /diff_drive_controller/cmd_vel_unstamped
    ros2 launch bot_bringup bringup.launch.py use_twist_mux:=false

What this does NOT start: SLAM, Nav2, teleop, RViz. Those are separate
launch files so you can restart them without power-cycling the robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("bot_bringup")

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_micro_ros = LaunchConfiguration("use_micro_ros")
    use_lidar = LaunchConfiguration("use_lidar")
    use_localization = LaunchConfiguration("use_localization")
    use_twist_mux = LaunchConfiguration("use_twist_mux")
    esp32_port = LaunchConfiguration("esp32_port")
    esp32_baud = LaunchConfiguration("esp32_baud")

    def include(name, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_share, "launch", name])
            ),
            **kwargs,
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="false",
            description="Replace the ESP32 with mock_components/GenericSystem."),
        DeclareLaunchArgument(
            "use_micro_ros", default_value="true",
            description="Start micro_ros_agent. Set false if you run it yourself."),
        DeclareLaunchArgument(
            "use_lidar", default_value="true"),
        DeclareLaunchArgument(
            "use_localization", default_value="true",
            description="Start the robot_localization EKF."),
        DeclareLaunchArgument(
            "use_twist_mux", default_value="true",
            description="Start twist_mux. If false, publish Twist directly to "
                        "/diff_drive_controller/cmd_vel_unstamped."),

        # These MUST be declared here and forwarded to sensors.launch.py.
        # A launch argument that is not declared at the level you pass it to is
        # silently ignored, which produces a stack that looks healthy - the
        # controllers report 'active' - while the agent sits on a device that
        # does not exist and nothing ever moves.
        DeclareLaunchArgument(
            "esp32_port", default_value="/dev/esp32",
            description="Serial device for the ESP32. /dev/ttyACM0 or "
                        "/dev/ttyUSB0 if the udev rules are not installed."),
        DeclareLaunchArgument(
            "esp32_baud", default_value="115200"),

        # 1. TF tree + /robot_description
        include("rsp.launch.py", launch_arguments={
            "use_mock_hardware": use_mock_hardware,
        }.items()),

        # 2. micro-ROS agent, lidar, IMU filter
        include("sensors.launch.py", launch_arguments={
            "use_micro_ros": use_micro_ros,
            "use_lidar": use_lidar,
            "esp32_port": esp32_port,
            "esp32_baud": esp32_baud,
        }.items()),

        # 3. ros2_control. Delayed so the micro-ROS agent has a moment to pick
        #    up the ESP32 first - the hardware interface copes fine either way,
        #    but the startup log is far easier to read in this order.
        TimerAction(period=3.0, actions=[
            include("control.launch.py", launch_arguments={
                "use_mock_hardware": use_mock_hardware,
                "use_twist_mux": use_twist_mux,
            }.items()),
        ]),

        # 4. EKF. Waits for diff_drive_controller to be publishing odometry.
        TimerAction(period=6.0, actions=[
            include("localization.launch.py",
                    condition=IfCondition(use_localization)),
        ]),
    ])
