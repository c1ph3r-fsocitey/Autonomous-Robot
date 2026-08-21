# Troubleshooting

Ordered roughly by how early in bring-up you will hit each one.

---

## The ESP32's LED blinks forever (never connects)

The firmware is running but cannot find the micro-ROS agent.

```bash
ls -l /dev/esp32                      # does the symlink exist?
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32 -b 115200 -v6
```

- **No `/dev/esp32`** → udev rules not installed, or matching the wrong device.
  See `udev/99-bot-serial.rules`.
- **`Permission denied`** → you are not in the `dialout` group. `sudo usermod -aG
  dialout $USER`, then log out and back in.
- **Agent runs but nothing appears** → baud mismatch. The firmware uses
  115200 (`SERIAL_BAUD` in `config.h`); the agent must match.
- **`Device or resource busy`** → something else has the port. A `pio device
  monitor` left open in another terminal is the usual suspect.

---

## `ros2 topic list` shows no `/wheel_state`

The agent connected but the ESP32 failed to create its entities. Restart the
agent with `-v6` and watch: if you see `create_participant` fail, the ESP32 ran
out of memory. That means something in `config.h` was raised past what the
default micro-ROS memory pool allows — the usual cause is adding publishers.

---

## Controllers refuse to start

```bash
ros2 control list_hardware_components
ros2 control list_controllers
```

- **`BotSystem` in `unconfigured`** → `bot_hardware` failed to load. Check the
  `ros2_control_node` log for a pluginlib error; usually the workspace was not
  re-sourced after building.
- **`joint_state_broadcaster` inactive** → the spawner ran before
  `controller_manager` was ready. Increase the `TimerAction` period in
  `bringup.launch.py`.
- **"Joint order mismatch"** → the `<joint>` entries in `ros2_control.xacro`
  are in the wrong order. Left wheel must be listed first.

---

## The robot drives, but backwards / spins when told to go straight

This is a sign problem, and it is always in the firmware, never in ROS.
Work through [`CALIBRATION.md`](CALIBRATION.md) §1 properly.

Quick discriminator:

| Behaviour | Meaning |
|---|---|
| Both wheels turn the wrong way | Both `INVERT_*` flags are wrong |
| Robot spins instead of driving | Exactly one `INVERT_*` is wrong |
| A wheel judders and howls | Motor and encoder disagree on direction — flip the **encoder** sign |
| Robot drives fine but odometry counts backwards | Encoder signs are inverted relative to motor signs |

---

## TF errors: "Could not transform" / "extrapolation into the future"

```bash
ros2 run tf2_tools view_frames        # writes frames.pdf
ros2 run tf2_ros tf2_echo odom base_footprint
```

- **Two publishers for `odom → base_footprint`** → `enable_odom_tf` got turned
  back on in `controllers.yaml`. It must stay `false`; the EKF owns that edge.
- **`map → odom` missing** → neither slam_toolbox nor amcl is running, or both
  crashed. Only ever run one of them.
- **Extrapolation errors under load** → the Pi is saturated. Move RViz to a
  laptop, and drop `controller_frequency` in `nav2_params.yaml`.

---

## The map is a spiral / corridors bend

Odometry, specifically heading. In order of likelihood:

1. `wheel_separation_multiplier` is wrong → CALIBRATION §4.
2. `TICKS_PER_REV` is wrong → CALIBRATION §2.
3. The IMU is not actually being fused. Check `/imu/data` is publishing and
   that `ekf_filter_node` is not logging "sensor timeout".
4. The IMU is mounted rotated, so its yaw axis is not vertical. `ros2 topic
   echo /imu/data_raw` and confirm `linear_acceleration.z ≈ 9.8` with the
   robot level.

---

## Map drifts when the robot is standing still

Gyro bias. `imu0_relative: true` in `ekf.yaml` zeroes the bias against the
first reading, so the robot must be **completely still** when the EKF starts.
If you launch while carrying it, restart the EKF once it is parked.

---

## SLAM eats the Pi and everything stutters

`slam_toolbox` is the expensive one. In `slam_mapping.yaml`:

- Raise `minimum_travel_distance` to `0.2` and `minimum_travel_heading` to `0.2`.
- Drop `resolution` to `0.05`.
- Drop `correlation_search_space_dimension` to `0.3`.
- Raise `map_update_interval` to `2.0`.

And run RViz somewhere else. RViz on the Pi will happily consume a whole core.

---

## Nav2 plans a path but the robot will not move

```bash
ros2 topic hz /cmd_vel_smoothed
ros2 topic hz /diff_drive_controller/cmd_vel_unstamped
```

- **First has data, second does not** → `twist_mux` is not running, or its
  output remap is wrong. Check `control.launch.py`.
- **Both have data, wheels do not turn** → `diff_drive_controller` is not
  active, or `use_stamped_vel` got set back to `true`.
- **Neither has data** → Nav2's lifecycle nodes did not activate. `ros2
  lifecycle list /controller_server`.

---

## Nav2 refuses to plan: "goal is in collision"

The inflated costmap has swallowed the free space. For a robot this small:

- `robot_radius: 0.11` — do not leave Nav2's default 0.22, it is for a machine
  twice this size.
- `inflation_radius: 0.22` with `cost_scaling_factor: 3.5`.
- `resolution: 0.03` on both costmaps, matching the SLAM map.

If a doorway still shows as blocked, look at the *map* rather than the params:
an A1M8 at grazing incidence to a wall produces a thick smeared return, and
that thickness is real as far as the costmap is concerned.

---

## Robot stops every few seconds during navigation

The ESP32's command watchdog (600 ms) is firing, meaning `/wheel_cmd` is not
arriving reliably.

- `ros2 topic hz /wheel_cmd` should show ~50 Hz.
- If it is lower or bursty, the Pi is CPU-starved.
- If it is 50 Hz but the robot still stalls, the serial link is dropping
  frames — try a shorter/better USB cable, and check the motor supply ground
  is properly tied to the ESP32 ground.

---

## Nothing works and you want to isolate the problem

Peel the stack back one layer at a time:

```bash
# 1. firmware alone — no ROS control involved
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32 -b 115200
ros2 run bot_bringup wheel_jog.py --left 5 --right 5 --duration 3

# 2. ros2_control, no sensors
ros2 launch bot_bringup bringup.launch.py use_lidar:=false use_localization:=false
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_key

# 3. everything except the robot
ros2 launch bot_bringup bringup.launch.py \
    use_mock_hardware:=true use_micro_ros:=false use_lidar:=false
```

If (3) works and (2) does not, the problem is hardware or firmware. If (3)
fails too, it is a launch/config problem and the robot is innocent.
