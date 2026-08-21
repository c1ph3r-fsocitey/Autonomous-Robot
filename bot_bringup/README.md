# ROS 2 stack for the Pi 4 + ESP32 differential drive robot

Target platform: **Ubuntu 22.04 / ROS 2 Humble** on a Raspberry Pi 4 (8 GB),
with an ESP32 running micro-ROS for motor control and IMU.

```
                      Raspberry Pi 4  (ROS 2 Humble)
  ┌────────────────────────────────────────────────────────────────┐
  │                                                                │
  │   Nav2  ──/cmd_vel_smoothed──►  twist_mux  ──►  diff_drive_    │
  │   teleop ─/cmd_vel_joy───────►              │   controller     │
  │                                             │        │         │
  │   slam_toolbox ──map→odom──┐                │   ros2_control   │
  │                            │                │        │         │
  │   robot_localization EKF ──┴─odom→base_footprint      │        │
  │        ▲            ▲                                 │        │
  │        │            │                          bot_hardware    │
  │   /diff_drive_   /imu/data                    (micro-ROS bridge)│
  │   controller/odom    ▲                                │        │
  │                imu_filter_madgwick                    │        │
  │                      ▲                                │        │
  │   rplidar_node  /imu/data_raw          /wheel_cmd ◄───┘        │
  │        │             │                 /wheel_state ──►        │
  └────────┼─────────────┼──────────────────────┼──────────────────┘
        /dev/rplidar     └───── micro_ros_agent ─┘ /dev/esp32
                                     │
                           ┌─────────┴──────────┐
                           │       ESP32        │
                           │  PID x2, encoders, │
                           │  TB6612FNG, MPU6050│
                           └────────────────────┘
```

---

## 1. Install dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-localization ros-humble-imu-filter-madgwick \
  ros-humble-twist-mux ros-humble-xacro \
  ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-joy ros-humble-teleop-twist-joy ros-humble-teleop-twist-keyboard \
  ros-humble-joint-state-publisher-gui
```

`micro_ros_agent` and `rplidar_ros` are already built in `~/microros_ws` and
`~/ros2_ws` respectively — nothing to do there.

## 2. Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/microros_ws/install/setup.bash        # for micro_ros_agent
colcon build --symlink-install \
  --packages-select bot_description bot_hardware bot_bringup bot_navigation
source install/setup.bash
```

A Pi 4 takes a few minutes on `bot_hardware` (it is the only C++ package).
Add these two lines to `~/.bashrc` so every new terminal is set up:

```bash
source ~/ros2_ws/install/setup.bash
source ~/microros_ws/install/setup.bash
```

## 3. Serial device names (do this once)

The ESP32 and the RPLidar both show up as `/dev/ttyUSB*` and the order is not
stable across reboots. Fix it:

```bash
sudo bash ~/ros2_ws/src/bot_bringup/udev/install_udev.sh
# unplug and replug both, then:
ls -l /dev/esp32 /dev/rplidar
```

If either symlink is missing, read the comment block at the bottom of
`udev/99-bot-serial.rules` — if both your devices use a CP2102 you need to
match on serial number, and the file tells you how to find it.

## 4. Flash the ESP32

See [`~/Arduino/bot_firmware_pio/README.md`](../../../../Arduino/bot_firmware_pio/README.md).
Short version: `cd ~/Arduino/bot_firmware_pio && pio run -t upload`.

## 5. Calibrate

**Do not skip this.** The placeholder `TICKS_PER_REV` in the firmware is a
guess, and everything above it — odometry, SLAM, Nav2 — is scaled by that one
number. See [`docs/CALIBRATION.md`](docs/CALIBRATION.md). Budget half an hour.

---

## Running it

Each block is its own terminal.

### Drive it around

```bash
# 1 — the robot
ros2 launch bot_bringup bringup.launch.py

# 2 — keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_key

# ...or a gamepad
ros2 launch bot_bringup teleop.launch.py
```

### Build a map

```bash
# 1 — the robot
ros2 launch bot_bringup bringup.launch.py
# 2 — SLAM
ros2 launch bot_bringup slam.launch.py
# 3 — teleop (as above)
# 4 — RViz (run this on a laptop if you have one, it is heavy for the Pi)
ros2 launch bot_bringup rviz.launch.py
```

Drive slowly. Cover every wall. Come back to where you started so loop closure
has something to work with. Then save:

```bash
mkdir -p ~/maps

# occupancy grid, for AMCL / Nav2
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab

# pose graph, for slam_toolbox localization mode
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: '/home/c1ph3r/maps/lab'}"
```

### Navigate a saved map

```bash
# 1 — the robot
ros2 launch bot_bringup bringup.launch.py
# 2 — localization (pick ONE)
ros2 launch bot_navigation amcl.launch.py map:=/home/c1ph3r/maps/lab.yaml
#   or:
ros2 launch bot_bringup slam.launch.py mode:=localization map_file:=/home/c1ph3r/maps/lab
# 3 — Nav2
ros2 launch bot_navigation navigation.launch.py
# 4 — RViz
ros2 launch bot_bringup rviz.launch.py config:=nav
```

In RViz: **2D Pose Estimate** to tell AMCL roughly where the robot is (skip
this with slam_toolbox localization), then **2D Goal Pose** to send it
somewhere.

### Explore and navigate at the same time

```bash
ros2 launch bot_bringup bringup.launch.py
ros2 launch bot_bringup slam.launch.py
ros2 launch bot_navigation navigation.launch.py
ros2 launch bot_bringup rviz.launch.py config:=nav
```

### No hardware at all

Everything except the actual motors runs on a desk:

```bash
ros2 launch bot_bringup bringup.launch.py \
    use_mock_hardware:=true use_micro_ros:=false use_lidar:=false
```

`mock_components/GenericSystem` pretends to be the ESP32 and integrates the
velocity commands, so TF, odometry and RViz all behave. Good for debugging
launch files without a robot on your desk.

---

## Package layout

| Package | What's in it |
|---|---|
| `bot_description` | URDF/xacro, RViz configs, standalone model viewer |
| `bot_hardware` | C++ `ros2_control` plugin bridging to the ESP32 over micro-ROS topics |
| `bot_bringup` | Launch files, controller/EKF/SLAM config, udev rules, calibration tools |
| `bot_navigation` | Nav2 parameters and navigation launch files |

Firmware lives outside the workspace, in `~/Arduino/bot_firmware_pio`
(PlatformIO) and `~/Arduino/bot_firmware` (Arduino IDE — same code).

## Key topics

| Topic | Type | Who publishes |
|---|---|---|
| `/wheel_cmd` | `Float64MultiArray` | `bot_hardware` → ESP32 |
| `/wheel_state` | `Float64MultiArray` | ESP32 → `bot_hardware` |
| `/wheel_ticks` | `Int32MultiArray` | ESP32 (calibration only) |
| `/imu/data_raw` | `sensor_msgs/Imu` | ESP32 |
| `/imu/data` | `sensor_msgs/Imu` | `imu_filter_madgwick` |
| `/scan` | `sensor_msgs/LaserScan` | `rplidar_node` |
| `/diff_drive_controller/odom` | `nav_msgs/Odometry` | `diff_drive_controller` (raw wheel odom) |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF (fused, and owns `odom`→`base_footprint`) |
| `/cmd_vel_joy`, `/cmd_vel_key` | `Twist` | teleop, highest priority in `twist_mux` |
| `/cmd_vel_smoothed` | `Twist` | Nav2, lowest priority |

## TF tree

```
map              <- slam_toolbox or amcl
 └─ odom         <- robot_localization EKF
     └─ base_footprint
         └─ base_link
             ├─ left_wheel / right_wheel   <- joint_state_broadcaster
             ├─ front_caster / rear_caster
             ├─ laser_frame
             └─ imu_link
```

Exactly one node publishes each edge. `diff_drive_controller` has
`enable_odom_tf: false` for precisely this reason — if you ever turn it on
you will have two publishers fighting over `odom → base_footprint` and TF
will flap between them.

## When something is wrong

See [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md).
