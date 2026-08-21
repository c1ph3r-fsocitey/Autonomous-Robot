// =============================================================================
//  config.h  --  ALL tunable constants for the robot firmware live here.
//  Edit this file, nothing else, when you rewire or recalibrate.
// =============================================================================
#pragma once

// -----------------------------------------------------------------------------
// 1. TB6612FNG MOTOR DRIVER PINS
// -----------------------------------------------------------------------------
// Channel A -> LEFT motor, Channel B -> RIGHT motor.
//
//   ESP32      TB6612FNG
//   GPIO 13 -> PWMA        left motor speed
//   GPIO 14 -> AIN1        left motor direction 1
//   GPIO 12 -> AIN2        left motor direction 2
//   GPIO 25 -> PWMB        right motor speed
//   GPIO 27 -> BIN1        right motor direction 1
//   GPIO 26 -> BIN2        right motor direction 2
//   3.3V    -> VCC,  battery+ -> VM,  all grounds common
//
// STBY is tied HIGH in hardware, so the firmware does not drive it.
// Set this to a GPIO number if you ever wire STBY to the ESP32 instead.
#define PIN_STBY      -1

#define PIN_L_PWM     13   // PWMA
#define PIN_L_IN1     14   // AIN1
#define PIN_L_IN2     12   // AIN2

#define PIN_R_PWM     25   // PWMB
#define PIN_R_IN1     27   // BIN1
#define PIN_R_IN2     26   // BIN2

// NOTE on GPIO12: it is a strapping pin (MTDI). The ESP32 samples it at reset
// to choose the flash voltage, and it must read LOW then. A TB6612 input is
// high-impedance so this is normally fine, but if the board ever refuses to
// boot after a power cycle, move AIN2 to a free pin (GPIO4, 5 or 23) and
// change it here. Symptom: no serial output at all, LED never blinks.

// If a wheel spins backwards relative to what you expect, flip its flag to 1.
#define INVERT_LEFT   0
#define INVERT_RIGHT  0

// -----------------------------------------------------------------------------
// 2. ENCODER PINS
// -----------------------------------------------------------------------------
//   Left  encoder A -> GPIO 34      Right encoder A -> GPIO 33
//   Left  encoder B -> GPIO 35      Right encoder B -> GPIO 32
//   Encoder GND -> ESP32 GND, encoder VCC -> 3.3V
#define PIN_ENC_L_A   34
#define PIN_ENC_L_B   35
#define PIN_ENC_R_A   33
#define PIN_ENC_R_B   32

// IMPORTANT - GPIO34 and GPIO35 are INPUT-ONLY and have NO internal pull-up or
// pull-down. The line below that enables weak pull-ups in setup() silently
// does nothing for them.
//
// Most N20 hall encoder boards drive their outputs push-pull, so this is fine.
// If it turns out yours are open-collector, the LEFT wheel will read zero
// ticks (or wildly noisy ones) while the right wheel behaves perfectly - the
// asymmetry is the tell. Fix it with a 10k resistor from each of GPIO34 and
// GPIO35 up to 3.3V.

// If encoder counts go the wrong way (wheel forward -> negative count),
// flip the corresponding flag instead of swapping wires.
// Both start at 0; determine them by hand, see docs/CALIBRATION.md section 1.
#define INVERT_ENC_LEFT   0
#define INVERT_ENC_RIGHT  0

// -----------------------------------------------------------------------------
// 3. I2C (MPU6050)
// -----------------------------------------------------------------------------
#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   22
#define I2C_FREQ_HZ   400000

// -----------------------------------------------------------------------------
// 4. DRIVE GEOMETRY  <<< CALIBRATE THESE >>>
// -----------------------------------------------------------------------------
// Encoder counts per revolution of the *output shaft* (i.e. of the wheel),
// counting all four quadrature edges.
//
//   TICKS_PER_REV = motor_PPR * 4 * gearbox_ratio
//
// Measured with:  ros2 run bot_bringup calibrate_encoders.py
//
// These are DELIBERATELY separate per wheel. Cheap N20s are often not a
// matched pair - the measured values below correspond to 7 PPR encoders on
// 250:1 (left) and 210:1 (right) gearboxes, which is a normal thing to end up
// with when the seller ships whatever is in the bin.
//
// If a re-measure gives you two values within a percent of each other, set
// both to the same number; nothing else needs to change.
#define TICKS_PER_REV_LEFT    6991.8f
#define TICKS_PER_REV_RIGHT   5886.4f

// Wheel radius in metres. 43 mm diameter wheels -> 0.0215 m.
// Do NOT trust the nominal number: measure the effective radius with
//     ros2 run bot_bringup calibrate_odom.py --mode linear
#define WHEEL_RADIUS_M  0.0215f

// -----------------------------------------------------------------------------
// 5. PWM
// -----------------------------------------------------------------------------
#define PWM_FREQ_HZ   20000   // 20 kHz -> above audible range, TB6612 handles it
#define PWM_RES_BITS  10      // 0..1023
#define PWM_MAX       ((1 << PWM_RES_BITS) - 1)

// Duty below which the motor just buzzes and does not turn. Anything smaller
// is snapped to zero. Find yours with: ros2 run bot_bringup wheel_jog.py
#define PWM_DEADBAND  0.06f   // fraction of full scale

// -----------------------------------------------------------------------------
// 6. VELOCITY PID (per wheel, operates on rad/s of the wheel)
// -----------------------------------------------------------------------------
// KFF is a feed-forward term: duty ~= KFF * target_rad_s. Set it to
// 1.0 / (max wheel rad/s at full duty) and the PID only has to trim.
#define PID_KFF   0.085f
#define PID_KP    0.045f
#define PID_KI    0.220f
#define PID_KD    0.0006f

#define PID_I_CLAMP 0.60f     // anti-windup clamp on the integral term (duty)

// Hard clamp on any incoming wheel command (rad/s). This is a safety limit,
// not the operating speed - the ROS side sets that in controllers.yaml.
//
// Measured with `wheel_jog.py --sweep`: both wheels track within 0.2% all the
// way to 12 rad/s with no sign of saturation, so 13 sits just above the
// verified range. On 43 mm wheels 12 rad/s = 0.258 m/s.
#define MAX_WHEEL_RAD_S  13.0f

// -----------------------------------------------------------------------------
// 7. LOOP RATES
// -----------------------------------------------------------------------------
#define CONTROL_HZ        50      // PID + odometry update
#define STATE_PUB_HZ      50      // /wheel_state
#define IMU_PUB_HZ        100     // /imu/data_raw
#define TICKS_PUB_HZ      10      // /wheel_ticks (calibration aid)

// If no /wheel_cmd arrives within this many ms, the motors are stopped.
#define CMD_TIMEOUT_MS    600

// -----------------------------------------------------------------------------
// 8. micro-ROS
// -----------------------------------------------------------------------------
#define UROS_NODE_NAME    "bot_firmware"
#define UROS_NAMESPACE    ""
#define UROS_DOMAIN_ID    0
#define SERIAL_BAUD       115200

#define IMU_FRAME_ID      "imu_link"

// -----------------------------------------------------------------------------
// 9. IMU noise (variance) -- used to fill the covariance matrices.
// MPU6050 datasheet-ish defaults; the EKF cares more about the ratio than
// the absolute value. Increase if the EKF trusts the IMU too much.
// -----------------------------------------------------------------------------
#define IMU_GYRO_VAR    0.0004f    // (rad/s)^2
#define IMU_ACCEL_VAR   0.0100f    // (m/s^2)^2

// Onboard LED used as a connection heartbeat. -1 to disable.
#define PIN_STATUS_LED  2
