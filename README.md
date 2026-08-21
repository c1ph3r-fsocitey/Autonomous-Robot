# c1ph3r-robot — ROS 2 differential drive robot

A 170 mm circular differential-drive robot. Raspberry Pi 4 (8 GB, Ubuntu 22.04,
ROS 2 Humble) running navigation and perception; an ESP32 running micro-ROS
handling motor control, encoders and IMU.

**Status:** drives cleanly under gamepad teleop. Calibrated, zero drift over a
full lap of a house. Lidar, SLAM and Nav2 still to come.

---

## Hardware

| Part | Detail |
|---|---|
| Compute | Raspberry Pi 4, 8 GB, Ubuntu 22.04, ROS 2 Humble |
| MCU | ESP32-D0WD-V3, micro-ROS over USB CDC (`/dev/ttyACM0`) |
| Motor driver | TB6612FNG, `STBY` tied to 3.3 V |
| Motors | 2× N20 gearmotors with quadrature encoders (**not a matched pair** — see below) |
| Wheels | 43 mm diameter, 180 mm separation |
| IMU | MPU6050, I²C |
| Lidar | RPLidar A1M8 (`/dev/ttyUSB0`) |

### ESP32 pin map

| Function | GPIO | | Function | GPIO |
|---|---|---|---|---|
| PWMA (left) | 13 | | PWMB (right) | 25 |
| AIN1 | 14 | | BIN1 | 27 |
| AIN2 | 12 | | BIN2 | 26 |
| Left enc A / B | 34 / 35 | | Right enc A / B | 33 / 32 |
| SDA / SCL | 21 / 22 | | Status LED | 2 |

GPIO 12 is a strapping pin (must read LOW at reset) and GPIO 34/35 are
input-only with no internal pull-ups. Both work here, but they are the first
things to suspect if the board stops booting or the left encoder starts
counting phantom edges.

### Calibrated values — measured, do not guess these

| Quantity | Value | Where |
|---|---|---|
| Left encoder | **6991.8** ticks/rev | `config.h` |
| Right encoder | **5886.4** ticks/rev | `config.h` |
| Wheel radius | 0.0215 m | `config.h`, `controllers.yaml` |
| Wheel separation | 0.180 m | `bot.urdf.xacro`, `controllers.yaml` |
| Verified wheel ceiling | 12 rad/s (0.258 m/s), ±0.2% tracking | measured |
| Operating limit | 0.22 m/s | `controllers.yaml` |

The two encoder constants differ by 18.8% because the gearboxes genuinely
differ — they land within 0.15% of `7 PPR × 4 × 250:1` and `7 PPR × 4 × 210:1`
respectively. Scaling each wheel separately is what makes the robot track
straight. **Do not average them.**

---

## Packages

| Package | Contents |
|---|---|
| `bot_description` | URDF/xacro, RViz configs |
| `bot_hardware` | C++ `ros2_control` plugin bridging to the ESP32 over micro-ROS topics |
| `bot_bringup` | Launch files, controller/EKF/SLAM config, udev rules, calibration tools |
| `bot_navigation` | Nav2 parameters and navigation launch |

Firmware lives in `~/Arduino/bot_firmware_pio` (PlatformIO) with a generated
Arduino IDE copy at `~/Arduino/bot_firmware`.

---

## Setting up a fresh machine

```bash
sudo apt install -y \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-localization ros-humble-imu-filter-madgwick \
  ros-humble-twist-mux ros-humble-xacro ros-humble-diagnostic-updater \
  ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-joy ros-humble-teleop-twist-joy ros-humble-teleop-twist-keyboard \
  python3-numpy

mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <this repo> .
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

# micro_ros_agent, built separately in ~/microros_ws
# https://micro.ros.org/docs/tutorials/core/first_application_linux/

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

chmod +x src/bot_bringup/scripts/*.py
sudo bash src/bot_bringup/udev/install_udev.sh
```

---

## Driving it

```bash
ros2 launch bot_bringup drive.launch.py
```

RT forward, LT reverse, left stick to steer. One Ctrl-C stops everything.

Useful arguments: `esp32_port:=/dev/esp32`, `use_lidar:=true`,
`use_twist_mux:=true`, `max_linear:=0.15`, `deadman_button:=4`.

## Diagnosing it

```bash
ros2 run bot_bringup check_stack.py
```

Walks the whole chain — micro-ROS link, hardware interface, controllers,
command topic, teleop, odometry/TF — and stops at the first break with the
command that fixes it.

## Other tools

```bash
ros2 run bot_bringup wheel_jog.py --left 3 --duration 2   # drive one wheel
ros2 run bot_bringup wheel_jog.py --sweep                 # PID tracking table
ros2 run bot_bringup calibrate_encoders.py                # ticks per rev
ros2 run bot_bringup calibrate_odom.py --mode linear      # wheel radius trim
ros2 run bot_bringup calibrate_odom.py --mode angular     # wheel separation trim
```

Full procedure in [`bot_bringup/docs/CALIBRATION.md`](bot_bringup/docs/CALIBRATION.md).
When something breaks, [`bot_bringup/docs/TROUBLESHOOTING.md`](bot_bringup/docs/TROUBLESHOOTING.md).

---

## Architecture

```
Nav2 ──/cmd_vel_smoothed──┐
teleop ──/cmd_vel_joy─────┴─► twist_mux ─► diff_drive_controller
                                                   │
slam_toolbox ──map→odom──┐                    ros2_control
                         │                         │
robot_localization EKF ──┴─ odom→base_footprint    │
     ▲          ▲                            bot_hardware
     │          │                                  │
/diff_drive_  /imu/data                      /wheel_cmd ─┐
controller/odom   ▲                       /wheel_state ◄─┤
             imu_filter_madgwick                         │
                  ▲                                      │
            /imu/data_raw ──── micro_ros_agent ──────────┘
                                     │
                              ESP32: PID ×2, encoders,
                              TB6612FNG, MPU6050
```

Exactly one node publishes each TF edge. `diff_drive_controller` has
`enable_odom_tf: false` because the EKF owns `odom → base_footprint` — turning
it back on gives you two publishers fighting and TF flaps between them.

### ESP32 ↔ Pi topics

| Direction | Topic | Type |
|---|---|---|
| Pi → ESP32 | `/wheel_cmd` | `std_msgs/Float64MultiArray` `[l_rad_s, r_rad_s]` |
| ESP32 → Pi | `/wheel_state` | `std_msgs/Float64MultiArray` `[l_pos, r_pos, l_vel, r_vel]` |
| ESP32 → Pi | `/wheel_ticks` | `std_msgs/Int32MultiArray` (calibration) |
| ESP32 → Pi | `/imu/data_raw` | `sensor_msgs/Imu` |

All best-effort QoS — `ros2 topic echo` needs `--qos-reliability best_effort`
or it will match nothing and look broken.

---

## Notes to future me

- **New script → rebuild.** `--symlink-install` links files at build time, so a
  newly added file is not picked up until `colcon build` runs again. Editing an
  existing file needs no rebuild.
- **Scripts need `chmod +x`.** `ros2 run` will not see a non-executable file,
  and the symlink inherits the source's permissions.
- **Never put `~/.platformio/penv/bin` on `PATH`.** Its `python3` shadows the
  system one, which hides apt-installed `numpy`, which breaks every rclpy node
  that imports `geometry_msgs`. Symlink just `pio` into `~/.local/bin`.
- **Motors coast when idle**, they do not brake. This is deliberate — running
  the PID against a zero target makes the wheels fight you during hand
  calibration.
- **The drive stack is frozen.** Pins, tick constants, PID gains, deadband and
  velocity limits are all measured and verified. Do not "improve" them without
  a reason and a re-measurement.
