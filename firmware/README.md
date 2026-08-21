# ESP32 firmware — PlatformIO, headless

PlatformIO Core is a plain command-line tool. The VSCode extension is just a
wrapper around it; you do not need an editor, a desktop, or a GUI of any kind.

This is the recommended toolchain for this project: it compiles micro-ROS from
source against your exact ESP32 core, which sidesteps the precompiled-library
version mismatches that make `micro_ros_arduino` fragile.

---

## 1. Install PlatformIO Core (once)

```bash
cd ~/Arduino/bot_firmware_pio
chmod +x setup_pio.sh
./setup_pio.sh
```

That installs the apt prerequisites, drops PlatformIO into its own virtualenv
at `~/.platformio/penv`, puts it on your `PATH` via `~/.bashrc`, and adds you
to the `dialout` group.

**Use the installer script, not `pip install platformio`.** The Pi's system
Python is PEP 668 managed, so a bare `pip install` either refuses or needs
`--break-system-packages` — and mixing PlatformIO's dependency tree into the
same environment ROS uses is a good way to break `colcon` later.

Open a new shell afterwards, or:

```bash
export PATH="$HOME/.local/bin:$PATH"
pio --version
```

> **Never put `~/.platformio/penv/bin` on your `PATH`.** That directory holds
> the virtualenv's own `python3`, which shadows `/usr/bin/python3` in every
> shell. A venv cannot see `/usr/lib/python3/dist-packages`, so apt-installed
> modules — `numpy` above all — disappear, and every rclpy node that imports
> `geometry_msgs` dies with `ModuleNotFoundError: No module named 'numpy'`.
> ROS's own packages keep importing fine, because they arrive via `PYTHONPATH`,
> so it presents as a ROS problem rather than a `PATH` problem.
>
> `setup_pio.sh` symlinks just the `pio` command into `~/.local/bin` instead.
> If you hit this, the fix is:
>
> ```bash
> sed -i '/platformio\/penv\/bin/d' ~/.bashrc
> ln -sf ~/.platformio/penv/bin/pio ~/.local/bin/pio
> # then open a new terminal
> ```

## 2. Build and flash

```bash
cd ~/Arduino/bot_firmware_pio
pio run -t upload
```

**The first build takes 15–25 minutes on a Pi 4.** It downloads the Xtensa
toolchain and then compiles the entire micro-ROS stack from source. It looks
like it has hung during `Building micro-ROS`; it has not. Watch it with
`pio run -v` if you want proof of life. Everything after that is ~20 seconds.

Useful variants:

```bash
pio run                            # compile only, no upload
pio run -t upload --upload-port /dev/ttyUSB0   # explicit port
pio run -t clean                   # clean the sketch, keep micro-ROS
pio run -t clean_microros          # nuke the micro-ROS build - use if it fails midway
pio system info                    # sanity check the install
```

## 3. Verify

```bash
# terminal 1
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32 -b 115200

# terminal 2
ros2 topic list | grep -E 'wheel|imu'
# /imu/data_raw  /wheel_cmd  /wheel_state  /wheel_ticks
```

The onboard LED goes **solid** when the agent connects; a slow blink means it
is still searching.

**Do not run `pio device monitor` on this port.** The firmware speaks the
micro-ROS binary protocol, not text — you will see garbage, and the monitor
holds the port open so the agent cannot get it. That is the number one cause
of `micro_ros_agent` reporting `Device or resource busy`.

---

## Layout

```
bot_firmware_pio/
├── platformio.ini      board, micro-ROS distro, library pins
├── setup_pio.sh        one-time PlatformIO install
└── src/
    ├── main.cpp        the firmware
    └── config.h        <- every tunable lives here
```

`~/Arduino/bot_firmware/` holds a generated `.ino` copy of the same code for
arduino-cli. If you are using PlatformIO, ignore it — editing both is how you
end up flashing the wrong one.

## Pin map (edit `config.h`, not the code)

| Function | ESP32 GPIO | Goes to |
|---|---|---|
| PWMA / AIN1 / AIN2 | 25 / 26 / 27 | TB6612FNG, **left** motor |
| PWMB / BIN1 / BIN2 | 32 / 33 / 14 | TB6612FNG, **right** motor |
| STBY | 13 | TB6612FNG |
| Left encoder A / B | 18 / 19 | N20 encoder |
| Right encoder A / B | 16 / 17 | N20 encoder |
| SDA / SCL | 21 / 22 | MPU6050 |
| Status LED | 2 | onboard |

**Power:** `VM` on the TB6612 takes the motor battery, `VCC` takes 3.3 V logic
from the ESP32. Tie every ground together — battery, ESP32, driver. A floating
ground is the single most common cause of "the encoders read garbage as soon
as the motors run".

Do not power the motors from the Pi's 5 V rail. N20 stall current will brown
out the Pi and you will spend an evening blaming ROS.

## ROS interface

| Direction | Topic | Type | Payload |
|---|---|---|---|
| ESP32 ← Pi | `/wheel_cmd` | `std_msgs/Float64MultiArray` | `[left_rad_s, right_rad_s]` |
| ESP32 → Pi | `/wheel_state` | `std_msgs/Float64MultiArray` | `[l_pos_rad, r_pos_rad, l_vel_rad_s, r_vel_rad_s]` |
| ESP32 → Pi | `/wheel_ticks` | `std_msgs/Int32MultiArray` | `[l_ticks, r_ticks]` (calibration only) |
| ESP32 → Pi | `/imu/data_raw` | `sensor_msgs/Imu` | accel + gyro, no orientation |

All best-effort QoS. The `bot_hardware` ros2_control plugin is the only thing
on the Pi that talks to `/wheel_cmd` and `/wheel_state`.

## Safety behaviour

No `/wheel_cmd` for 600 ms → motors coast to a stop. Agent disconnects →
motors stop immediately and the firmware goes back to looking for the agent,
no reboot needed on either side.

---

## When it goes wrong

| Symptom | Fix |
|---|---|
| `pio: command not found` | New shell, or `export PATH="$HOME/.platformio/penv/bin:$PATH"` |
| Build stalls at `Building micro-ROS` | It is working. 15–25 min on a Pi 4. `pio run -v` to watch |
| micro-ROS build fails partway | `pio run -t clean_microros && pio run -t upload` |
| `Failed to connect to ESP32: Timed out` | Hold **BOOT** as the upload starts. A 10 µF cap from EN to GND fixes it permanently |
| Upload fails at 921600 baud | Lower `upload_speed` in `platformio.ini` to `460800` or `115200` |
| `Permission denied: /dev/ttyUSB0` | Not in `dialout` yet — log out and back in after `setup_pio.sh` |
| `Device or resource busy` | The agent or a serial monitor still holds the port |
| `could not open port /dev/esp32` | udev rules not installed: `sudo bash ~/ros2_ws/src/bot_bringup/udev/install_udev.sh` |
| Out of disk mid-build | micro-ROS build tree is ~1.5 GB. `df -h ~` |

---

## After flashing: calibrate

`TICKS_PER_REV` in `config.h` is a **placeholder** — everything downstream
(odometry, SLAM, Nav2) is scaled by it.

```bash
ros2 run bot_bringup calibrate_encoders.py
```

Full procedure: `~/ros2_ws/src/bot_bringup/docs/CALIBRATION.md`.
