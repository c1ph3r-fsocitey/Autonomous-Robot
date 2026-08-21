"""Racing-style joystick teleop: triggers for throttle, left stick to steer.

    # normal - feeds twist_mux, so Nav2 can be running too
    ros2 launch bot_bringup teleop_race.launch.py

    # twist_mux broken or not installed: talk straight to the controller
    ros2 launch bot_bringup teleop_race.launch.py \\
        cmd_topic:=/diff_drive_controller/cmd_vel_unstamped

    # different pad layout
    ros2 launch bot_bringup teleop_race.launch.py steer_axis:=0 \\
        throttle_axis:=5 brake_axis:=2

Squeeze and release both triggers once after starting - they arm on first
sight of a rest value, which is what stops the robot bolting at half throttle
because joy_node reports an untouched trigger as 0.0.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cmd_topic = LaunchConfiguration("cmd_topic")

    # Launch arguments arrive as strings. declare_parameter() in the node
    # fixed each parameter's type, so they have to be coerced here or the node
    # rejects them at startup with a type-mismatch error.
    def as_int(name):
        return ParameterValue(LaunchConfiguration(name), value_type=int)

    def as_float(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument(
            "cmd_topic", default_value="/cmd_vel_joy",
            description="Where to publish Twist. /cmd_vel_joy goes through "
                        "twist_mux; use /diff_drive_controller/"
                        "cmd_vel_unstamped to bypass it."),
        DeclareLaunchArgument("joy_device_id", default_value="0"),
        DeclareLaunchArgument("steer_axis", default_value="0"),
        DeclareLaunchArgument("throttle_axis", default_value="5"),
        DeclareLaunchArgument("brake_axis", default_value="2"),
        DeclareLaunchArgument(
            "deadman_button", default_value="-1",
            description="Button that must be held to drive. -1 to disable."),
        DeclareLaunchArgument("max_linear", default_value="0.22"),
        DeclareLaunchArgument("max_angular", default_value="1.50"),

        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[{
                "device_id": as_int("joy_device_id"),
                "deadzone": 0.05,
                "autorepeat_rate": 25.0,
                # Report untouched triggers as 1.0 (released) rather than 0.0.
                # The teleop node arms them defensively anyway, but this makes
                # them usable immediately instead of after the first squeeze.
                "default_trig_val": True,
            }],
            output="both",
        ),

        Node(
            package="bot_bringup",
            executable="joy_race_teleop.py",
            name="joy_race_teleop",
            parameters=[{
                "steer_axis": as_int("steer_axis"),
                "throttle_axis": as_int("throttle_axis"),
                "brake_axis": as_int("brake_axis"),
                "deadman_button": as_int("deadman_button"),
                "max_linear": as_float("max_linear"),
                "max_angular": as_float("max_angular"),
            }],
            remappings=[("cmd_vel", cmd_topic)],
            output="both",
        ),
    ])
