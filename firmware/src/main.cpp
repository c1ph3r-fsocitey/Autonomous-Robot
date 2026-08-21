// =============================================================================
//  bot_firmware  --  ESP32 micro-ROS node for a 2-wheel differential drive base
//
//  Hardware:  ESP32 dev board
//             TB6612FNG dual H-bridge  ->  2x N20 gearmotors with quadrature
//             encoders, MPU6050 IMU on I2C
//
//  ROS graph (all under the micro-ROS agent):
//    sub   /wheel_cmd     std_msgs/Float64MultiArray  [left_rad_s, right_rad_s]
//    pub   /wheel_state   std_msgs/Float64MultiArray  [l_pos_rad, r_pos_rad,
//                                                      l_vel_rad_s, r_vel_rad_s]
//    pub   /wheel_ticks   std_msgs/Int32MultiArray    [l_ticks, r_ticks]
//    pub   /imu/data_raw  sensor_msgs/Imu
//
//  Everything the ROS 2 side needs is in those four topics; the Pi runs a
//  ros2_control hardware interface (bot_hardware) that speaks to them.
//
//  Portability: builds under PlatformIO (micro_ros_platformio) and under the
//  Arduino IDE (micro_ros_arduino) without edits.
// =============================================================================

#include <Arduino.h>

#if __has_include(<micro_ros_platformio.h>)
  #include <micro_ros_platformio.h>
#else
  #include <micro_ros_arduino.h>
#endif

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/imu.h>

#include <ESP32Encoder.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "config.h"

// -----------------------------------------------------------------------------
// Small helpers
// -----------------------------------------------------------------------------
#define RCCHECK(fn)      { rcl_ret_t _rc = (fn); if (_rc != RCL_RET_OK) { return false; } }
#define RCSOFTCHECK(fn)  { rcl_ret_t _rc = (fn); (void)_rc; }

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static const float TWO_PI_F = 6.28318530718f;

// =============================================================================
//  Motor driver (TB6612FNG)
// =============================================================================
class Motor {
public:
  Motor(uint8_t pwm, uint8_t in1, uint8_t in2, uint8_t channel, bool invert)
    : pwm_(pwm), in1_(in1), in2_(in2), ch_(channel), invert_(invert) {}

  void begin() {
    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(pwm_, PWM_FREQ_HZ, PWM_RES_BITS);
#else
    ledcSetup(ch_, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(pwm_, ch_);
#endif
    stop();
  }

  // duty in [-1.0, 1.0]
  void set(float duty) {
    if (invert_) duty = -duty;
    duty = clampf(duty, -1.0f, 1.0f);

    // Anything under 1% is a genuine "stop" request.
    if (fabsf(duty) < 0.01f) { stop(); return; }

    // Dead-band compensation: an N20 will not budge below PWM_DEADBAND, so
    // remap (0,1] onto [PWM_DEADBAND,1] instead of throwing small values away.
    const float mag = PWM_DEADBAND + (1.0f - PWM_DEADBAND) * fabsf(duty);

    const bool forward = duty > 0.0f;
    digitalWrite(in1_, forward ? HIGH : LOW);
    digitalWrite(in2_, forward ? LOW  : HIGH);
    write_(static_cast<uint32_t>(clampf(mag, 0.0f, 1.0f) * PWM_MAX));
  }

  // Both inputs low = coast. Both high would be brake; coast is gentler on N20s.
  void stop() {
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);
    write_(0);
  }

private:
  void write_(uint32_t d) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pwm_, d);
#else
    ledcWrite(ch_, d);
#endif
  }
  uint8_t pwm_, in1_, in2_, ch_;
  bool invert_;
};

// =============================================================================
//  Velocity PID  (target and measurement in rad/s of the wheel)
// =============================================================================
class VelocityPID {
public:
  void reset() { integral_ = 0.0f; prev_error_ = 0.0f; }

  float update(float target, float measured, float dt) {
    const float error = target - measured;

    // Feed-forward carries most of the load; PID only trims.
    const float ff = PID_KFF * target;

    integral_ += error * dt;
    integral_ = clampf(integral_, -PID_I_CLAMP / (PID_KI + 1e-6f),
                                   PID_I_CLAMP / (PID_KI + 1e-6f));

    const float derivative = (dt > 1e-6f) ? (error - prev_error_) / dt : 0.0f;
    prev_error_ = error;

    float out = ff + PID_KP * error + PID_KI * integral_ + PID_KD * derivative;

    // If we are saturated and the integral is pushing further into saturation,
    // bleed it back off (conditional anti-windup).
    if (out > 1.0f || out < -1.0f) {
      const float sat = clampf(out, -1.0f, 1.0f);
      integral_ -= (out - sat) / (PID_KI + 1e-6f);
      out = sat;
    }
    return out;
  }

private:
  float integral_    = 0.0f;
  float prev_error_  = 0.0f;
};

// =============================================================================
//  Globals
// =============================================================================
static Motor motor_left (PIN_L_PWM, PIN_L_IN1, PIN_L_IN2, 0, INVERT_LEFT);
static Motor motor_right(PIN_R_PWM, PIN_R_IN1, PIN_R_IN2, 1, INVERT_RIGHT);

static ESP32Encoder enc_left;
static ESP32Encoder enc_right;

static VelocityPID pid_left, pid_right;

static Adafruit_MPU6050 mpu;
static bool imu_ok = false;

// Commanded wheel velocities (rad/s), written by the subscription callback.
static volatile float cmd_left_rad_s  = 0.0f;
static volatile float cmd_right_rad_s = 0.0f;
static volatile uint32_t last_cmd_ms  = 0;

// Integrated wheel positions (rad) and last measured velocities.
static double pos_left_rad  = 0.0;
static double pos_right_rad = 0.0;
static float  vel_left_rad_s  = 0.0f;
static float  vel_right_rad_s = 0.0f;

static int64_t last_ticks_left  = 0;
static int64_t last_ticks_right = 0;
static uint32_t last_control_us = 0;

// -----------------------------------------------------------------------------
// micro-ROS entities
// -----------------------------------------------------------------------------
static rcl_allocator_t   allocator;
static rclc_support_t    support;
static rcl_node_t        node;
static rclc_executor_t   executor;

static rcl_subscription_t sub_wheel_cmd;
static rcl_publisher_t    pub_wheel_state;
static rcl_publisher_t    pub_wheel_ticks;
static rcl_publisher_t    pub_imu;

static rcl_timer_t timer_state;
static rcl_timer_t timer_imu;
static rcl_timer_t timer_ticks;

static std_msgs__msg__Float64MultiArray msg_cmd;
static std_msgs__msg__Float64MultiArray msg_state;
static std_msgs__msg__Int32MultiArray   msg_ticks;
static sensor_msgs__msg__Imu            msg_imu;

// Static backing storage so micro-ROS never has to malloc at runtime.
static double  cmd_buf[2];
static double  state_buf[4];
static int32_t ticks_buf[2];

static std_msgs__msg__MultiArrayDimension cmd_dims[2];
static std_msgs__msg__MultiArrayDimension state_dims[4];
static std_msgs__msg__MultiArrayDimension ticks_dims[2];
static char cmd_dim_labels[2][24];
static char state_dim_labels[4][24];
static char ticks_dim_labels[2][24];

// Agent connection state machine
enum class AgentState { WAITING, AVAILABLE, CONNECTED, DISCONNECTED };
static AgentState agent_state = AgentState::WAITING;

// =============================================================================
//  Static allocation of MultiArray messages
// =============================================================================
static void init_dim_array(std_msgs__msg__MultiArrayDimension *dims,
                           char labels[][24], size_t capacity) {
  for (size_t i = 0; i < capacity; ++i) {
    labels[i][0]        = '\0';
    dims[i].label.data     = labels[i];
    dims[i].label.size     = 0;
    dims[i].label.capacity = 24;
    dims[i].size   = 0;
    dims[i].stride = 0;
  }
}

static void init_messages() {
  // /wheel_cmd  (incoming -- must be pre-allocated or micro-ROS will drop it)
  init_dim_array(cmd_dims, cmd_dim_labels, 2);
  msg_cmd.layout.dim.data     = cmd_dims;
  msg_cmd.layout.dim.size     = 0;
  msg_cmd.layout.dim.capacity = 2;
  msg_cmd.layout.data_offset  = 0;
  msg_cmd.data.data     = cmd_buf;
  msg_cmd.data.size     = 0;
  msg_cmd.data.capacity = 2;

  // /wheel_state (outgoing)
  init_dim_array(state_dims, state_dim_labels, 4);
  msg_state.layout.dim.data     = state_dims;
  msg_state.layout.dim.size     = 0;
  msg_state.layout.dim.capacity = 4;
  msg_state.layout.data_offset  = 0;
  msg_state.data.data     = state_buf;
  msg_state.data.size     = 4;
  msg_state.data.capacity = 4;

  // /wheel_ticks (outgoing)
  init_dim_array(ticks_dims, ticks_dim_labels, 2);
  msg_ticks.layout.dim.data     = ticks_dims;
  msg_ticks.layout.dim.size     = 0;
  msg_ticks.layout.dim.capacity = 2;
  msg_ticks.layout.data_offset  = 0;
  msg_ticks.data.data     = ticks_buf;
  msg_ticks.data.size     = 2;
  msg_ticks.data.capacity = 2;

  // /imu/data_raw -- fixed size, but frame_id is a string we must back ourselves
  static char imu_frame[] = IMU_FRAME_ID;
  msg_imu.header.frame_id.data     = imu_frame;
  msg_imu.header.frame_id.size     = strlen(imu_frame);
  msg_imu.header.frame_id.capacity = sizeof(imu_frame);

  // MPU6050 gives no absolute orientation. -1 in element 0 is the ROS
  // convention for "this message contains no orientation estimate".
  msg_imu.orientation.x = 0.0; msg_imu.orientation.y = 0.0;
  msg_imu.orientation.z = 0.0; msg_imu.orientation.w = 1.0;
  for (int i = 0; i < 9; ++i) {
    msg_imu.orientation_covariance[i]         = 0.0;
    msg_imu.angular_velocity_covariance[i]    = 0.0;
    msg_imu.linear_acceleration_covariance[i] = 0.0;
  }
  msg_imu.orientation_covariance[0] = -1.0;
  msg_imu.angular_velocity_covariance[0]    = IMU_GYRO_VAR;
  msg_imu.angular_velocity_covariance[4]    = IMU_GYRO_VAR;
  msg_imu.angular_velocity_covariance[8]    = IMU_GYRO_VAR;
  msg_imu.linear_acceleration_covariance[0] = IMU_ACCEL_VAR;
  msg_imu.linear_acceleration_covariance[4] = IMU_ACCEL_VAR;
  msg_imu.linear_acceleration_covariance[8] = IMU_ACCEL_VAR;
}

// =============================================================================
//  Time
// =============================================================================
// Fills a builtin_interfaces Time from the agent-synchronised epoch so that the
// ROS 2 side sees timestamps on the same clock as everything else.
static void fill_stamp(builtin_interfaces__msg__Time *stamp) {
  const int64_t ns = rmw_uros_epoch_nanos();
  stamp->sec     = static_cast<int32_t>(ns / 1000000000LL);
  stamp->nanosec = static_cast<uint32_t>(ns % 1000000000LL);
}

// =============================================================================
//  Callbacks
// =============================================================================
static void wheel_cmd_callback(const void *msgin) {
  const auto *m = static_cast<const std_msgs__msg__Float64MultiArray *>(msgin);
  if (m->data.size >= 2) {
    cmd_left_rad_s  = clampf(static_cast<float>(m->data.data[0]), -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S);
    cmd_right_rad_s = clampf(static_cast<float>(m->data.data[1]), -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S);
    last_cmd_ms = millis();
  }
}

static void state_timer_callback(rcl_timer_t *, int64_t) {
  // Positions are float64 on the wire on purpose: they accumulate forever and
  // float32 would start losing sub-millimetre resolution after a few hundred
  // metres of driving, which shows up as odometry drift.
  state_buf[0] = pos_left_rad;
  state_buf[1] = pos_right_rad;
  state_buf[2] = vel_left_rad_s;
  state_buf[3] = vel_right_rad_s;
  RCSOFTCHECK(rcl_publish(&pub_wheel_state, &msg_state, NULL));
}

static void ticks_timer_callback(rcl_timer_t *, int64_t) {
  ticks_buf[0] = static_cast<int32_t>(last_ticks_left);
  ticks_buf[1] = static_cast<int32_t>(last_ticks_right);
  RCSOFTCHECK(rcl_publish(&pub_wheel_ticks, &msg_ticks, NULL));
}

static void imu_timer_callback(rcl_timer_t *, int64_t) {
  if (!imu_ok) return;

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  fill_stamp(&msg_imu.header.stamp);

  // Adafruit returns m/s^2 and rad/s already, in the sensor's own frame.
  // If your MPU6050 is mounted rotated, remap the axes HERE (and only here)
  // so that +X is robot-forward, +Y is robot-left, +Z is up.
  msg_imu.linear_acceleration.x = a.acceleration.x;
  msg_imu.linear_acceleration.y = a.acceleration.y;
  msg_imu.linear_acceleration.z = a.acceleration.z;

  msg_imu.angular_velocity.x = g.gyro.x;
  msg_imu.angular_velocity.y = g.gyro.y;
  msg_imu.angular_velocity.z = g.gyro.z;

  RCSOFTCHECK(rcl_publish(&pub_imu, &msg_imu, NULL));
}

// =============================================================================
//  Control loop (runs from loop(), not from an rcl timer, so it keeps the
//  motors sane even while the agent is away)
// =============================================================================
static void control_step() {
  const uint32_t now_us = micros();
  float dt = (now_us - last_control_us) * 1e-6f;
  last_control_us = now_us;
  if (dt <= 0.0f || dt > 0.5f) dt = 1.0f / CONTROL_HZ;   // first call / overflow

  // --- read encoders -------------------------------------------------------
  int64_t t_left  = enc_left.getCount();
  int64_t t_right = enc_right.getCount();
#if INVERT_ENC_LEFT
  t_left = -t_left;
#endif
#if INVERT_ENC_RIGHT
  t_right = -t_right;
#endif

  const int64_t d_left  = t_left  - last_ticks_left;
  const int64_t d_right = t_right - last_ticks_right;
  last_ticks_left  = t_left;
  last_ticks_right = t_right;

  // Per-wheel scaling: the two gearboxes are not necessarily identical, and
  // using one average for both puts a permanent curve into the odometry.
  const float d_rad_left  = d_left  * (TWO_PI_F / TICKS_PER_REV_LEFT);
  const float d_rad_right = d_right * (TWO_PI_F / TICKS_PER_REV_RIGHT);

  pos_left_rad  += d_rad_left;
  pos_right_rad += d_rad_right;

  // Light low-pass on velocity -- N20 encoders are noisy at low speed.
  const float alpha = 0.4f;
  vel_left_rad_s  = (1 - alpha) * vel_left_rad_s  + alpha * (d_rad_left  / dt);
  vel_right_rad_s = (1 - alpha) * vel_right_rad_s + alpha * (d_rad_right / dt);

  // --- watchdog ------------------------------------------------------------
  float target_l = cmd_left_rad_s;
  float target_r = cmd_right_rad_s;
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    target_l = 0.0f;
    target_r = 0.0f;
  }

  // --- PID -----------------------------------------------------------------
  // Nothing commanded: COAST. Do not run the PID against a zero target.
  //
  // The obvious-looking alternative - "keep regulating to zero" - makes the
  // motor actively resist any wheel movement it did not ask for, because a
  // hand-turned wheel reads as a velocity error. The integrator winds up the
  // longer you hold it, so the wheel gets progressively stiffer. That makes
  // hand calibration impossible and it is why encoder calibration felt like
  // fighting the robot.
  //
  // Coasting is also the safer default here: an N20 gearbox has enough
  // internal friction to stop a 1 kg robot promptly on its own, so there is
  // nothing to gain from electrical braking.
  if (target_l == 0.0f && target_r == 0.0f) {
    pid_left.reset();
    pid_right.reset();
    motor_left.stop();
    motor_right.stop();
    return;
  }

  motor_left.set (pid_left.update (target_l, vel_left_rad_s,  dt));
  motor_right.set(pid_right.update(target_r, vel_right_rad_s, dt));
}

// =============================================================================
//  micro-ROS entity lifecycle
// =============================================================================
static bool create_entities() {
  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, UROS_DOMAIN_ID));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  RCCHECK(rclc_node_init_default(&node, UROS_NODE_NAME, UROS_NAMESPACE, &support));

  // --- subscription: best effort, we only care about the newest command ----
  RCCHECK(rclc_subscription_init_best_effort(
      &sub_wheel_cmd, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "wheel_cmd"));

  // --- publishers ----------------------------------------------------------
  RCCHECK(rclc_publisher_init_best_effort(
      &pub_wheel_state, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "wheel_state"));

  RCCHECK(rclc_publisher_init_best_effort(
      &pub_wheel_ticks, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "wheel_ticks"));

  RCCHECK(rclc_publisher_init_best_effort(
      &pub_imu, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu/data_raw"));

  // --- timers --------------------------------------------------------------
  RCCHECK(rclc_timer_init_default(&timer_state, &support,
      RCL_MS_TO_NS(1000 / STATE_PUB_HZ), state_timer_callback));
  RCCHECK(rclc_timer_init_default(&timer_imu, &support,
      RCL_MS_TO_NS(1000 / IMU_PUB_HZ), imu_timer_callback));
  RCCHECK(rclc_timer_init_default(&timer_ticks, &support,
      RCL_MS_TO_NS(1000 / TICKS_PUB_HZ), ticks_timer_callback));

  // --- executor: 1 subscription + 3 timers ---------------------------------
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_wheel_cmd, &msg_cmd,
                                         &wheel_cmd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_state));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_ticks));

  // Line up our clock with the agent's so timestamps are usable by TF.
  rmw_uros_sync_session(1000);

  return true;
}

static void destroy_entities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_subscription_fini(&sub_wheel_cmd, &node);
  rcl_publisher_fini(&pub_wheel_state, &node);
  rcl_publisher_fini(&pub_wheel_ticks, &node);
  rcl_publisher_fini(&pub_imu, &node);
  rcl_timer_fini(&timer_state);
  rcl_timer_fini(&timer_imu);
  rcl_timer_fini(&timer_ticks);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// =============================================================================
//  setup / loop
// =============================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  set_microros_serial_transports(Serial);

#if PIN_STATUS_LED >= 0
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);
#endif

  // --- motor driver --------------------------------------------------------
#if PIN_STBY >= 0
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);          // take the TB6612 out of standby
#endif
  // With PIN_STBY = -1 the driver's STBY pin is tied HIGH in hardware and the
  // firmware must not touch it. Driving a pin that is actually wired to
  // something else - PWMA, say - leaves that motor at full duty with its
  // direction pins ignored, which is a very confusing thing to debug.
  motor_left.begin();
  motor_right.begin();

  // --- encoders ------------------------------------------------------------
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc_left.attachFullQuad(PIN_ENC_L_A, PIN_ENC_L_B);
  enc_right.attachFullQuad(PIN_ENC_R_A, PIN_ENC_R_B);
  enc_left.clearCount();
  enc_right.clearCount();

  // --- IMU -----------------------------------------------------------------
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
  imu_ok = mpu.begin(0x68, &Wire);
  if (imu_ok) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  }

  init_messages();

  last_control_us = micros();
  last_cmd_ms     = millis();

  delay(1000);   // let the agent come up if both booted together
}

void loop() {
  const uint32_t now = millis();

  // --- fixed-rate control, independent of the agent ------------------------
  static uint32_t next_control_ms = 0;
  if ((int32_t)(now - next_control_ms) >= 0) {
    next_control_ms = now + (1000 / CONTROL_HZ);
    control_step();
  }

  // --- agent connection state machine --------------------------------------
  static uint32_t next_ping_ms = 0;

  switch (agent_state) {
    case AgentState::WAITING:
      if ((int32_t)(now - next_ping_ms) >= 0) {
        next_ping_ms = now + 500;
        agent_state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
                        ? AgentState::AVAILABLE : AgentState::WAITING;
      }
      break;

    case AgentState::AVAILABLE:
      agent_state = create_entities() ? AgentState::CONNECTED
                                      : AgentState::WAITING;
      if (agent_state == AgentState::WAITING) destroy_entities();
      break;

    case AgentState::CONNECTED:
      if ((int32_t)(now - next_ping_ms) >= 0) {
        next_ping_ms = now + 1000;
        if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
          agent_state = AgentState::DISCONNECTED;
          break;
        }
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
      break;

    case AgentState::DISCONNECTED:
      destroy_entities();
      // Agent went away mid-drive: stop before anything else.
      cmd_left_rad_s  = 0.0f;
      cmd_right_rad_s = 0.0f;
      motor_left.stop();
      motor_right.stop();
      agent_state = AgentState::WAITING;
      break;
  }

#if PIN_STATUS_LED >= 0
  // Solid = connected, slow blink = looking for the agent.
  digitalWrite(PIN_STATUS_LED,
               agent_state == AgentState::CONNECTED ? HIGH : ((now / 400) & 1));
#endif
}
