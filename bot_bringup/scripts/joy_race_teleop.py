#!/usr/bin/env python3
"""Racing-style joystick teleop: triggers for throttle, left stick for steering.

    RT  forward        LT  reverse/brake
    left stick X       steering
    (optional) hold a deadman button

Compared to teleop_twist_joy this gives you analogue throttle and an expo
steering curve, which is far nicer to drive. Everything is a ROS parameter, so
you can retune it live with `ros2 param set` without restarting.

    # normal: feed twist_mux, which arbitrates against Nav2
    ros2 launch bot_bringup teleop_race.launch.py

    # bypass twist_mux entirely
    ros2 launch bot_bringup teleop_race.launch.py \\
        cmd_topic:=/diff_drive_controller/cmd_vel_unstamped

Based on race_teleop_pro.py, with three changes that matter on real hardware:

  1. TRIGGER ARMING. joy_node reports an untouched trigger axis as 0.0, not
     1.0. The original maps that to (1 - 0)/2 = 0.5, so the robot launches at
     half throttle the instant you start the node. Each trigger here is
     ignored until it has been seen at rest (>= 0.9) at least once.

  2. JOY WATCHDOG. If the gamepad disconnects or its battery dies, the
     original keeps publishing the last target forever and the robot drives
     into a wall. This one zeroes the command if /joy goes quiet.

  3. LIMITS THAT MATCH THE ROBOT. max_linear defaults to 0.22 m/s. The
     measured wheel ceiling is 12 rad/s = 0.258 m/s, and we leave ~15%
     headroom. The original's 0.5 would just be clamped by
     diff_drive_controller, making the top half of the trigger travel do
     nothing.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyRaceTeleop(Node):

    def __init__(self):
        super().__init__("joy_race_teleop")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        p = self.declare_parameter

        # Axis / button indices. Defaults are the Xbox 360 layout that the
        # Evo Fox Elite X2 reports in X-input mode (8 axes, 11 buttons).
        p("steer_axis", 0)          # left stick, horizontal
        p("throttle_axis", 5)       # RT
        p("brake_axis", 2)          # LT
        p("deadman_button", -1)     # -1 disables the deadman entirely

        # Motion limits. Keep these at or below what the robot can do -
        # diff_drive_controller clamps anything higher, which just makes the
        # top of the stick/trigger travel feel dead.
        p("max_linear", 0.22)       # m/s
        p("max_angular", 1.50)      # rad/s

        # Feel
        p("smoothing", 0.20)        # 0..1 per tick; lower = softer
        p("deadzone", 0.10)         # steering stick deadzone
        p("throttle_deadzone", 0.05)
        p("expo", 2.5)              # >1 = finer control near centre
        p("turn_scale_moving", 0.70)   # calmer steering at speed
        p("turn_scale_static", 1.20)   # snappier pivot when stopped

        p("publish_rate", 50.0)
        p("joy_timeout", 0.5)       # seconds of silence before we stop

        self._load_params()
        self.add_on_set_parameters_callback(self._on_params)

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0

        # A trigger is not trusted until we have seen it at rest once.
        self.throttle_armed = False
        self.brake_armed = False

        self.last_joy_time = None
        self.warned_no_joy = False

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        # joy_node publishes best-effort on some setups and reliable on
        # others; best-effort here matches either without complaint.
        joy_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Joy, "joy", self.joy_callback, joy_qos)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.update)

        self.get_logger().info(
            f"Ready. throttle=axes[{self.throttle_axis}] "
            f"brake=axes[{self.brake_axis}] steer=axes[{self.steer_axis}] "
            f"max {self.max_linear:.2f} m/s / {self.max_angular:.2f} rad/s"
        )
        # teleop_race.launch.py starts joy_node with default_trig_val:=true, so
        # the triggers report "released" immediately and arm on the first
        # message - no user action needed. This only matters if you run this
        # node against a joy_node started some other way.
        self.get_logger().info(
            "Throttle arms automatically. If it does not respond, squeeze and "
            "release both triggers once.")

    # ----------------------------------------------------------------------
    def _load_params(self):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.steer_axis = int(g("steer_axis"))
        self.throttle_axis = int(g("throttle_axis"))
        self.brake_axis = int(g("brake_axis"))
        self.deadman_button = int(g("deadman_button"))
        self.max_linear = float(g("max_linear"))
        self.max_angular = float(g("max_angular"))
        self.smoothing = float(g("smoothing"))
        self.deadzone = float(g("deadzone"))
        self.throttle_deadzone = float(g("throttle_deadzone"))
        self.expo = float(g("expo"))
        self.turn_scale_moving = float(g("turn_scale_moving"))
        self.turn_scale_static = float(g("turn_scale_static"))
        self.publish_rate = float(g("publish_rate"))
        self.joy_timeout = float(g("joy_timeout"))

    def _on_params(self, _params):
        from rcl_interfaces.msg import SetParametersResult
        # Re-read everything; simpler than tracking which one changed and
        # there is no cost worth caring about at this rate.
        try:
            self._load_params()
            return SetParametersResult(successful=True)
        except Exception as exc:                      # noqa: BLE001
            return SetParametersResult(successful=False, reason=str(exc))

    # ----------------------------------------------------------------------
    @staticmethod
    def _apply_expo(value, expo):
        """Expo curve that preserves sign. |value| <= 1 assumed."""
        if value >= 0.0:
            return float(value ** expo)
        return float(-((-value) ** expo))

    def _read_trigger(self, msg, axis, armed_attr):
        """Return 0..1 for a trigger, handling the un-armed 0.0 case.

        joy_node reports an untouched trigger as 0.0 and a released one as
        +1.0. Those map to half throttle and zero throttle respectively, so
        we refuse to believe a trigger until it has reported a rest value.
        """
        if axis < 0 or axis >= len(msg.axes):
            return 0.0

        raw = float(msg.axes[axis])

        if not getattr(self, armed_attr):
            if raw >= 0.9:                    # seen at rest - now trustworthy
                setattr(self, armed_attr, True)
            return 0.0

        value = (1.0 - raw) / 2.0
        return 0.0 if value < self.throttle_deadzone else min(value, 1.0)

    # ----------------------------------------------------------------------
    def joy_callback(self, msg):
        self.last_joy_time = self.get_clock().now()
        if self.warned_no_joy:
            self.get_logger().info("Joystick back.")
            self.warned_no_joy = False

        # --- deadman ------------------------------------------------------
        if self.deadman_button >= 0:
            if (self.deadman_button >= len(msg.buttons) or
                    not msg.buttons[self.deadman_button]):
                self.target_linear = 0.0
                self.target_angular = 0.0
                return

        # --- throttle -----------------------------------------------------
        throttle = self._read_trigger(msg, self.throttle_axis, "throttle_armed")
        brake = self._read_trigger(msg, self.brake_axis, "brake_armed")
        self.target_linear = (throttle - brake) * self.max_linear

        # --- steering -----------------------------------------------------
        steer = 0.0
        if 0 <= self.steer_axis < len(msg.axes):
            steer = float(msg.axes[self.steer_axis])

        if abs(steer) < self.deadzone:
            steer = 0.0
        else:
            # Rescale so the curve starts at the edge of the deadzone rather
            # than jumping discontinuously as you leave it.
            sign = 1.0 if steer > 0 else -1.0
            steer = sign * (abs(steer) - self.deadzone) / (1.0 - self.deadzone)
            steer = self._apply_expo(steer, self.expo)

        angular = steer * self.max_angular

        # Calmer steering while moving, snappier pivot when stopped.
        if abs(self.target_linear) > 0.05:
            angular *= self.turn_scale_moving
        else:
            angular *= self.turn_scale_static

        self.target_angular = max(-self.max_angular,
                                  min(self.max_angular, angular))

    # ----------------------------------------------------------------------
    def update(self):
        # --- watchdog -----------------------------------------------------
        if self.last_joy_time is None:
            self._publish(0.0, 0.0)
            return

        age = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        if age > self.joy_timeout:
            if not self.warned_no_joy:
                self.get_logger().warn(
                    f"No /joy for {age:.1f}s - stopping. "
                    "Is the gamepad still connected?")
                self.warned_no_joy = True
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.current_linear = 0.0
            self.current_angular = 0.0
            self._publish(0.0, 0.0)
            return

        # --- exponential smoothing ----------------------------------------
        self.current_linear += self.smoothing * (
            self.target_linear - self.current_linear)
        self.current_angular += self.smoothing * (
            self.target_angular - self.current_angular)

        if abs(self.current_linear) < 0.01:
            self.current_linear = 0.0
        if abs(self.current_angular) < 0.01:
            self.current_angular = 0.0

        self._publish(self.current_linear, self.current_angular)

    def _publish(self, linear, angular):
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.pub.publish(twist)


def main():
    rclpy.init()
    node = JoyRaceTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the robot stopped, not coasting on the last command.
        for _ in range(10):
            node._publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
