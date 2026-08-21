from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('my_robot')
    rplidar_pkg = get_package_share_directory('rplidar_ros')

    return LaunchDescription([

        # ── Robot description (URDF) ──────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': open(
                    os.path.join(pkg, 'urdf', 'robot.urdf.xml')).read()
            }]
        ),

        # ── micro-ROS agent (ESP32) ───────────────────────────
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0', '--baudrate', '115200'],
            output='screen'
        ),

        # ── RPLidar ──────────────────────────────────────────
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }],
            output='screen'
        ),

        # ── EKF (wheel odom → filtered odom + TF) ────────────
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(pkg, 'config', 'ekf.yaml')],
            remappings=[('odometry/filtered', '/odometry/filtered')]
        ),

        # ── Joystick + teleop ─────────────────────────────────
        Node(
            package='joy',
            executable='joy_node',
            parameters=[{'dev': '/dev/input/js0'}]
        ),

        Node(
            package='my_robot',    # your teleop script
            executable='race_teleop.py',
            output='screen'
        ),
    ])
