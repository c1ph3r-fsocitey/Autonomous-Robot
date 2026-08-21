# Calibration

Four things, in this order. Each one depends on the previous ones being right,
so resist the urge to skip ahead.

1. Motor direction and encoder sign
2. Encoder ticks per revolution
3. Effective wheel radius (linear odometry)
4. Effective wheel separation (angular odometry)

Then, optionally: PID tracking and lidar alignment.

Total time: about half an hour, and it is the difference between a robot that
maps a room and a robot that draws a spiral.

---

## 0. Before you start

Put the robot on a stand so the wheels spin freely. A stack of books works.

```bash
# terminal 1 — just the agent, no controllers
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32 -b 115200
```

Confirm the ESP32's status LED goes **solid**. Then:

```bash
ros2 topic list | grep -E 'wheel|imu'
# expect: /imu/data_raw  /wheel_cmd  /wheel_state  /wheel_ticks
```

If the LED blinks forever, the agent is not reaching the board — wrong device
path, or the ESP32 is not running the firmware.

---

## 1. Motor direction and encoder sign

```bash
ros2 run bot_bringup wheel_jog.py --left 5 --duration 3
```

Watch the **left** wheel. It must turn in the direction that would drive the
robot **forward**.

| What you see | Fix in `config.h` |
|---|---|
| Left wheel turns backwards | `INVERT_LEFT` → flip 0/1 |
| Right wheel turns backwards | `INVERT_RIGHT` → flip 0/1 |
| Wrong wheel moved entirely | Swap the A/B motor outputs on the TB6612FNG |

Then check the encoder sign:

```bash
ros2 topic echo /wheel_state --once
```

`data[2]` (left velocity) should be **positive** while the wheel is being
commanded forward. If it is negative, flip `INVERT_ENC_LEFT`. Same for
`data[3]` and `INVERT_ENC_RIGHT`.

Repeat for `--right 5`. Re-flash after each change. Do not move on until both
wheels turn forward and both report positive velocity.

> A wheel that turns but reports zero velocity means the encoder is not being
> read at all — check the A/B wiring and that the encoder has power. A wheel
> that oscillates violently means the motor and encoder disagree about
> direction, and the PID is chasing its own tail: flip the encoder sign, not
> the motor sign.

---

## 2. Encoder ticks per revolution

The single most important number in the stack.

```bash
ros2 run bot_bringup calibrate_encoders.py
```

Follow the prompts: mark each wheel, turn it by hand through a known number of
revolutions, and the tool prints ticks per revolution. Ten revolutions gives
plenty of averaging.

Put the answer in `config.h`:

```c
#define TICKS_PER_REV   2800.0f     // <- your measured value
```

Re-flash.

**Sanity check.** For an N20 the number should be roughly
`motor_PPR × 4 × gear_ratio`. If you measure 1400 and expected 2800, your
encoder is being read in 2x quadrature somewhere, or the gearbox is half what
the listing claimed. If left and right differ by more than ~3%, suspect a
loose connector or electrical noise rather than a real difference — a shared
ground between the motor supply and the ESP32 is the usual culprit.

---

## 3. Effective wheel radius

43 mm nominal is never 43 mm in practice: the tyre compresses under load and
moulding tolerances are loose.

Start the full stack, give yourself 3 m of clear floor, then:

```bash
ros2 run bot_bringup calibrate_odom.py --mode linear --distance 2.0
```

The robot drives until *its own odometry* says 2 m. You measure how far it
actually went, and the tool prints a correction factor.

Apply it in **one** of two places:

- **Firmware** (preferred, fixes the source): multiply `WHEEL_RADIUS_M` in
  `config.h` by the factor and re-flash.
- **ROS side** (no re-flash): set `left_wheel_radius_multiplier` and
  `right_wheel_radius_multiplier` in `bot_bringup/config/controllers.yaml`.

Not both. Re-run the test to confirm you land within 1%.

> A correction factor outside roughly 0.8–1.3 almost always means
> `TICKS_PER_REV` is wrong, not that the wheel is a strange size. Go back to
> step 2.

---

## 4. Effective wheel separation

The nominal 180 mm is the distance between wheel centres. What the kinematics
actually care about is the distance between contact patches, which differs
because the tyres deform and the wheels are never perfectly perpendicular.

```bash
ros2 run bot_bringup calibrate_odom.py --mode angular --turns 5
```

Line up a mark on the chassis with a mark on the floor. The robot spins until
its odometry reads 5 rotations; you report how far it actually turned.

Apply the factor to `wheel_separation_multiplier` in
`bot_bringup/config/controllers.yaml`.

Intuition: if it **over**-rotates (actual > odom), the real separation is
larger than configured, so the multiplier goes **up**.

This is the parameter that matters most for SLAM. A robot whose heading
odometry is 5% off will produce a map with a persistent curve in every
straight corridor.

---

## 5. PID tracking (optional, but do it if the robot feels sluggish)

```bash
# stop bringup first - it will fight you for /wheel_cmd
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32 -b 115200
ros2 run bot_bringup wheel_jog.py --sweep
```

You get a table of commanded vs achieved wheel speed. Reading it:

| Symptom | Cause | Fix in `config.h` |
|---|---|---|
| Big error at low speed only | Motor not overcoming stiction | Raise `PWM_DEADBAND` |
| Robot creeps when commanded 0 | Deadband compensation too aggressive | Lower `PWM_DEADBAND` |
| Error grows with speed, always negative | Saturating | Lower `MAX_WHEEL_RAD_S`, or raise `PID_KFF` |
| Slow to reach target, no overshoot | Under-damped integral | Raise `PID_KI` |
| Oscillates around target | Too much gain | Lower `PID_KP`, then `PID_KI` |
| Buzzes audibly at rest | PWM frequency in the audible band | Raise `PWM_FREQ_HZ` |

Set `MAX_WHEEL_RAD_S` to about 85% of whatever the sweep shows the wheels can
actually reach. Asking for a speed the motors cannot deliver just winds up the
integrator and makes the robot lurch when the load changes.

Then update the matching limits in `controllers.yaml`:

```
max_wheel_rad_s × wheel_radius = linear.x.max_velocity
```

---

## 6. Lidar alignment (optional)

If the map looks rotated relative to the robot — walls appear at 90° to where
they are — the lidar's zero-degree mark is not pointing forward.

Park the robot with a flat wall directly in front of it, then:

```bash
ros2 launch bot_bringup rviz.launch.py
```

With **Fixed Frame** set to `base_footprint`, the wall in `/scan` should show
up directly ahead (+X). If it is off by some angle, set `lidar_yaw` in
`bot_description/urdf/bot.urdf.xacro` to that angle in radians and rebuild.

Common values: `0`, `${pi}` (lidar mounted backwards), `${pi/2}`,
`${-pi/2}`.

---

## Quick reference: where each number lives

| Quantity | File | Parameter |
|---|---|---|
| Encoder ticks/rev | `bot_firmware/config.h` | `TICKS_PER_REV` |
| Wheel radius | `bot_firmware/config.h` | `WHEEL_RADIUS_M` |
| Wheel radius (ROS trim) | `controllers.yaml` | `*_wheel_radius_multiplier` |
| Wheel separation | `bot.urdf.xacro` + `controllers.yaml` | `wheel_separation` |
| Wheel separation (trim) | `controllers.yaml` | `wheel_separation_multiplier` |
| Motor/encoder direction | `bot_firmware/config.h` | `INVERT_*` |
| PID gains | `bot_firmware/config.h` | `PID_*` |
| Max speed | `bot_firmware/config.h` + `controllers.yaml` | `MAX_WHEEL_RAD_S`, `linear.x.max_velocity` |
| Lidar mounting | `bot.urdf.xacro` | `lidar_x/y/z`, `lidar_yaw` |
| IMU mounting | `bot.urdf.xacro` | `imu_x/y/z`, `imu_joint` rpy |

Anywhere a number appears twice, the URDF is the source of truth for geometry
and the firmware is the source of truth for anything the ESP32 needs to do
its own maths.
