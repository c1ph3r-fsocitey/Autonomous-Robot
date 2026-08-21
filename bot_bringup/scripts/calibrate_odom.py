#!/usr/bin/env python3
"""Calibrate wheel radius and wheel separation against reality.

Two tests, run in this order:

  1. LINEAR  - drives straight for a set odometry distance. You measure how
               far it actually went with a tape measure. The ratio corrects
               the effective wheel radius (tyre compression and manufacturing
               tolerance mean the printed diameter is never quite right).

  2. ANGULAR - spins in place for a set number of odometry rotations. You
               observe how far it actually turned. The ratio corrects
               wheel_separation, which is almost always slightly off because
               the real contact patch is not at the centre of the tyre.

Do LINEAR first: the angular test depends on the radius being right.

    ros2 run bot_bringup calibrate_odom.py --mode linear  --distance 2.0
    ros2 run bot_bringup calibrate_odom.py --mode angular --turns 5

Requires bringup.launch.py to be running. Both tests read
/diff_drive_controller/odom - the RAW wheel odometry - deliberately, not the
EKF output, because we are trying to measure the wheels' error, and the EKF
exists to hide exactly that error.

Give yourself a few metres of clear floor. Tape a marker to the chassis and
mark the floor at the start and end.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class OdomCalibrator(Node):
    def __init__(self, odom_topic, cmd_topic):
        super().__init__("odom_calibrator")
        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._cb, 10)
        self.pose = None       # (x, y, yaw)

    def _cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def wait_for_odom(self, timeout=15.0):
        waited = 0.0
        while self.pose is None and waited < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            waited += 0.1
        return self.pose is not None

    def drive(self, linear, angular):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.pub.publish(t)

    def stop(self):
        for _ in range(15):
            self.drive(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)


def run_linear(node, target_distance, speed):
    print()
    print("=" * 68)
    print("  LINEAR TEST")
    print("=" * 68)
    print(f"The robot will drive forward until its odometry reads "
          f"{target_distance:.2f} m.")
    print("Mark the floor at the front edge of the robot NOW.")
    input("Press ENTER to start (Ctrl-C stops it)... ")

    x0, y0, _ = node.pose
    travelled = 0.0
    deadline = time.time() + (target_distance / max(speed, 0.01)) * 4 + 15

    while travelled < target_distance and rclpy.ok() and time.time() < deadline:
        node.drive(speed, 0.0)
        rclpy.spin_once(node, timeout_sec=0.05)
        x, y, _ = node.pose
        travelled = math.hypot(x - x0, y - y0)

    node.stop()
    time.sleep(0.5)
    rclpy.spin_once(node, timeout_sec=0.2)
    x, y, _ = node.pose
    travelled = math.hypot(x - x0, y - y0)

    print()
    print(f"  odometry says it went: {travelled:.3f} m")
    print("  Mark the floor at the front edge again and measure the gap.")
    actual = input("  Actual distance travelled (metres): ").strip()
    try:
        actual = float(actual)
    except ValueError:
        print("  Not a number. Aborting.")
        return

    if actual <= 0 or travelled <= 0:
        print("  Need a positive distance. Aborting.")
        return

    ratio = actual / travelled
    print()
    print("=" * 68)
    print(f"  correction factor = actual / odom = {ratio:.4f}")
    print()
    if abs(ratio - 1.0) < 0.01:
        print("  Within 1%. Leave the config alone.")
        return

    print("  Apply it EITHER in the firmware (preferred - fixes the source):")
    print(f"      WHEEL_RADIUS_M in config.h  ->  multiply by {ratio:.4f}")
    print()
    print("  OR in bot_bringup/config/controllers.yaml (no re-flash needed):")
    print(f"      left_wheel_radius_multiplier:  {ratio:.4f}")
    print(f"      right_wheel_radius_multiplier: {ratio:.4f}")
    print()
    print("  Do NOT do both. Re-run this test afterwards to confirm.")
    if ratio > 1.3 or ratio < 0.77:
        print()
        print("  That is a big correction. Before applying it, double-check")
        print("  TICKS_PER_REV - a wrong tick count looks exactly like this.")


def run_angular(node, turns, ang_speed):
    print()
    print("=" * 68)
    print("  ANGULAR TEST")
    print("=" * 68)
    print(f"The robot will spin in place until its odometry reads "
          f"{turns:g} full rotations.")
    print("Line up a mark on the robot with a mark on the floor NOW.")
    input("Press ENTER to start (Ctrl-C stops it)... ")

    target_yaw = turns * 2.0 * math.pi
    _, _, prev = node.pose
    accumulated = 0.0
    deadline = time.time() + (target_yaw / max(ang_speed, 0.05)) * 4 + 15

    while accumulated < target_yaw and rclpy.ok() and time.time() < deadline:
        node.drive(0.0, ang_speed)
        rclpy.spin_once(node, timeout_sec=0.05)
        _, _, yaw = node.pose
        d = yaw - prev
        # unwrap
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        accumulated += abs(d)
        prev = yaw

    node.stop()
    time.sleep(0.5)

    print()
    print(f"  odometry says it turned: {accumulated / (2*math.pi):.3f} rotations")
    print("  How far did it ACTUALLY turn? Count full turns, then estimate the")
    print("  leftover angle in degrees (e.g. 4 turns + 30 deg -> 4.083).")
    actual = input("  Actual rotations: ").strip()
    try:
        actual = float(actual)
    except ValueError:
        print("  Not a number. Aborting.")
        return

    odom_turns = accumulated / (2 * math.pi)
    if actual <= 0 or odom_turns <= 0:
        print("  Need a positive number of rotations. Aborting.")
        return

    ratio = actual / odom_turns
    print()
    print("=" * 68)
    print(f"  correction factor = actual / odom = {ratio:.4f}")
    print()
    if abs(ratio - 1.0) < 0.02:
        print("  Within 2%. Leave the config alone.")
        return

    print("  In bot_bringup/config/controllers.yaml, set:")
    print(f"      wheel_separation_multiplier: {ratio:.4f}")
    print()
    print("  Intuition: if the robot over-rotates (actual > odom), the real")
    print("  wheel separation is larger than configured, so the multiplier")
    print("  goes up. Re-run to confirm.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["linear", "angular"], required=True)
    parser.add_argument("--distance", type=float, default=2.0,
                        help="linear test target, metres")
    parser.add_argument("--speed", type=float, default=0.12,
                        help="linear test speed, m/s")
    parser.add_argument("--turns", type=float, default=5.0,
                        help="angular test target, full rotations")
    parser.add_argument("--ang-speed", type=float, default=0.8,
                        help="angular test speed, rad/s")
    parser.add_argument("--odom-topic", default="/diff_drive_controller/odom")
    parser.add_argument("--cmd-topic", default="/cmd_vel_key",
                        help="twist_mux input to publish on")
    args = parser.parse_args()

    rclpy.init()
    node = OdomCalibrator(args.odom_topic, args.cmd_topic)

    print(f"Waiting for {args.odom_topic} ...")
    if not node.wait_for_odom():
        print()
        print(f"Nothing on {args.odom_topic} after 15 s.")
        print("Is bringup.launch.py running, and did diff_drive_controller")
        print("actually activate?  ros2 control list_controllers")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    try:
        if args.mode == "linear":
            run_linear(node, args.distance, args.speed)
        else:
            run_angular(node, args.turns, args.ang_speed)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
