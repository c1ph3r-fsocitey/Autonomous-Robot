#!/usr/bin/env python3
"""Drive the wheels directly, bypassing ros2_control. Bring-up and PID tuning.

This talks straight to the ESP32's /wheel_cmd topic, so it works before
diff_drive_controller is configured and it isolates firmware problems from
ROS-side problems.

    # spin the left wheel forward at 5 rad/s for 3 seconds
    ros2 run bot_bringup wheel_jog.py --left 5 --duration 3

    # both wheels, forward
    ros2 run bot_bringup wheel_jog.py --left 5 --right 5 --duration 3

    # step through speeds and report how well the PID tracks each one
    ros2 run bot_bringup wheel_jog.py --sweep

Stop anything else that publishes /wheel_cmd first (i.e. don't run
bringup.launch.py's control stack at the same time) or you will be fighting
the controller for the topic.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray


class WheelJog(Node):
    def __init__(self):
        super().__init__("wheel_jog")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(Float64MultiArray, "/wheel_cmd", qos)
        self.state = None
        self.create_subscription(
            Float64MultiArray, "/wheel_state", self._cb, qos)

    def _cb(self, msg):
        if len(msg.data) >= 4:
            self.state = list(msg.data)

    def send(self, left, right):
        msg = Float64MultiArray()
        msg.data = [float(left), float(right)]
        self.pub.publish(msg)

    def hold(self, left, right, duration, rate_hz=25.0):
        """Publish continuously - the firmware stops after 600 ms of silence."""
        period = 1.0 / rate_hz
        end = time.time() + duration
        while time.time() < end and rclpy.ok():
            self.send(left, right)
            rclpy.spin_once(self, timeout_sec=period)

    def stop(self):
        for _ in range(10):
            self.send(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)


def do_sweep(node, max_rad_s, step):
    print()
    print(f"{'target':>10} {'left':>10} {'right':>10} {'L err%':>8} {'R err%':>8}")
    print("-" * 50)

    target = step
    while target <= max_rad_s + 1e-9:
        node.hold(target, target, 2.0)          # settle
        samples = []
        end = time.time() + 1.0
        while time.time() < end and rclpy.ok():
            node.send(target, target)
            rclpy.spin_once(node, timeout_sec=0.04)
            if node.state:
                samples.append((node.state[2], node.state[3]))

        if samples:
            avg_l = sum(s[0] for s in samples) / len(samples)
            avg_r = sum(s[1] for s in samples) / len(samples)
            err_l = (avg_l - target) / target * 100.0
            err_r = (avg_r - target) / target * 100.0
            print(f"{target:>10.2f} {avg_l:>10.2f} {avg_r:>10.2f} "
                  f"{err_l:>+8.1f} {err_r:>+8.1f}")
        else:
            print(f"{target:>10.2f} {'no data':>10}")

        target += step

    node.stop()
    print()
    print("Reading the table:")
    print("  * Errors within a few % across the range: the PID is fine.")
    print("  * Large positive error at low speed, near zero at high speed:")
    print("      PWM_DEADBAND is too high in config.h.")
    print("  * Large NEGATIVE error that grows with speed: the wheel is")
    print("      saturating - lower MAX_WHEEL_RAD_S, or PID_KFF is too small.")
    print("  * Left and right differ consistently: mechanical, not electrical.")
    print("      Check for a rubbing wheel or a tight gearbox.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", type=float, default=0.0,
                        help="left wheel target, rad/s")
    parser.add_argument("--right", type=float, default=0.0,
                        help="right wheel target, rad/s")
    parser.add_argument("--duration", type=float, default=2.0, help="seconds")
    parser.add_argument("--sweep", action="store_true",
                        help="step through speeds and report tracking error")
    parser.add_argument("--max", type=float, default=12.0,
                        help="top speed for --sweep, rad/s")
    parser.add_argument("--step", type=float, default=2.0,
                        help="speed increment for --sweep, rad/s")
    args = parser.parse_args()

    rclpy.init()
    node = WheelJog()

    try:
        if args.sweep:
            do_sweep(node, args.max, args.step)
        else:
            print(f"left={args.left} rad/s  right={args.right} rad/s  "
                  f"for {args.duration}s")
            node.hold(args.left, args.right, args.duration)
            node.stop()
            if node.state:
                print(f"  last measured: left={node.state[2]:.2f} "
                      f"right={node.state[3]:.2f} rad/s")
            else:
                print("  no /wheel_state received - is the agent running?")
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
