#!/usr/bin/env python3
"""Measure encoder counts per wheel revolution.

This is the single most important number in the whole stack. Everything
downstream - odometry, SLAM, Nav2 - is scaled by it, and the datasheet value
for an N20 is very often wrong (gearbox ratios are nominal, and "11 PPR" may
mean 11 pulses or 11 edges depending on who wrote the listing).

Usage:
    # with the micro-ROS agent running
    ros2 run bot_bringup calibrate_encoders.py

It first watches the stationary wheels for electrical noise, then asks you to
turn each wheel by hand through a whole number of revolutions.

The two wheels are measured and reported SEPARATELY. Cheap N20s are frequently
not a matched pair, and averaging two different gearboxes bakes a permanent
curve into your odometry.
"""

import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32MultiArray

# 7 PPR is by far the most common N20 magnetic encoder; 4 edges per pulse.
COMMON_PPR = (7, 11, 12, 13)
COMMON_RATIOS = (30, 50, 75, 100, 150, 200, 210, 250, 298, 380, 1000)


class EncoderCalibrator(Node):
    def __init__(self):
        super().__init__("encoder_calibrator")

        # The ESP32 publishes best-effort; a reliable subscription would never
        # match it and you would sit here watching nothing happen.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.ticks = None
        self.create_subscription(Int32MultiArray, "/wheel_ticks", self._cb, qos)

    def _cb(self, msg):
        if len(msg.data) >= 2:
            self.ticks = (int(msg.data[0]), int(msg.data[1]))

    def wait_for_data(self, timeout=15.0):
        waited = 0.0
        while self.ticks is None and waited < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            waited += 0.1
        return self.ticks is not None


def guess_ratio(ticks_per_rev):
    """Find the closest standard PPR x 4 x gear-ratio combination."""
    best = None
    for ppr in COMMON_PPR:
        for ratio in COMMON_RATIOS:
            nominal = ppr * 4 * ratio
            err = abs(nominal - ticks_per_rev) / ticks_per_rev * 100.0
            if best is None or err < best[0]:
                best = (err, ppr, ratio, nominal)
    return best


def drift_check(node, seconds=8.0):
    """Watch the stationary wheels. Any movement here is electrical noise.

    Floating encoder inputs are the classic cause. On an ESP32, GPIO34-39 are
    input-only and have NO internal pull-ups, so an encoder wired there will
    happily count phantom edges picked up from the motor leads.
    """
    print()
    print("=" * 68)
    print("  NOISE CHECK")
    print("=" * 68)
    print(f"Do not touch the wheels for {seconds:.0f} seconds.")
    input("Press ENTER when your hands are off... ")

    start = node.ticks
    time.sleep(seconds)
    end = node.ticks

    d_left = end[0] - start[0]
    d_right = end[1] - start[1]

    print(f"  left drift : {d_left:+d} ticks")
    print(f"  right drift: {d_right:+d} ticks")

    if d_left == 0 and d_right == 0:
        print("  Clean. Any difference between the wheels is mechanical, not noise.")
        return True

    print()
    print("  !! Counts moved while the wheels were still. That is electrical")
    print("     noise, and it will inflate whichever side is affected.")
    print()
    print("     Most likely cause: encoder inputs on GPIO34-39. Those pins are")
    print("     input-only and have NO internal pull-ups, so the firmware's")
    print("     pull-up setting silently does nothing for them.")
    print()
    print("     Fix, either one:")
    print("       * 10k resistor from each affected pin to 3.3V, or")
    print("       * move that encoder to GPIO 4/5/16/17/18/19/23 and update")
    print("         PIN_ENC_* in config.h")
    print()
    print("     Also worth checking: motor ground and ESP32 ground must be")
    print("     tied together, and encoder wires should not run alongside")
    print("     motor leads.")
    return False


def measure(node, wheel_index, wheel_name):
    print()
    print("=" * 68)
    print(f"  {wheel_name.upper()} WHEEL")
    print("=" * 68)
    print("The wheel should turn freely - the motors coast when idle.")
    revs = input("How many revolutions will you turn it? [10] ").strip()
    try:
        revs = float(revs) if revs else 10.0
    except ValueError:
        revs = 10.0

    input("Mark the wheel, then press ENTER to zero the count... ")
    start = node.ticks[wheel_index]

    print(f"Now turn the {wheel_name} wheel FORWARD exactly {revs:g} revolutions.")
    input("Press ENTER when the mark is back where it started... ")
    end = node.ticks[wheel_index]

    delta = end - start
    if delta == 0:
        print("  !! No change in tick count.")
        print("     Check the encoder wiring and that the ESP32 is publishing:")
        print("       ros2 topic echo /wheel_ticks --qos-reliability best_effort")
        return None

    per_rev = abs(delta) / revs

    print()
    print(f"  ticks counted : {delta}")
    print(f"  direction     : {'forward (+)' if delta > 0 else 'BACKWARD (-)'}")
    print(f"  TICKS PER REV : {per_rev:.1f}")

    err, ppr, ratio, nominal = guess_ratio(per_rev)
    if err < 2.0:
        print(f"  closest match : {ppr} PPR x 4 x {ratio}:1 = {nominal} "
              f"({err:.2f}% off)")
    else:
        print(f"  closest match : {ppr} PPR x 4 x {ratio}:1 = {nominal} "
              f"({err:.1f}% off - no clean match, re-measure)")

    if delta < 0:
        print()
        print(f"  !! The count went negative when you turned the wheel forward.")
        print(f"     Flip INVERT_ENC_{wheel_name.upper()} in config.h.")
    return per_rev


def main():
    rclpy.init()
    node = EncoderCalibrator()
    executor = SingleThreadedExecutor()
    spin_thread = None
    try:
        print("Waiting for /wheel_ticks ...")
        if not node.wait_for_data():
            print()
            print("Nothing on /wheel_ticks after 15 s. Check that:")
            print("  * micro_ros_agent is running")
            print("  * the ESP32's status LED is SOLID, not blinking")
            print("  * ros2 topic list | grep wheel")
            return 1

        # Keep callbacks flowing while the main thread blocks on input().
        # An executor (rather than a bare rclpy.spin) is what lets us shut the
        # background thread down cleanly instead of aborting on exit.
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        print("Got data.")
        clean = drift_check(node)

        left = measure(node, 0, "left")
        right = measure(node, 1, "right")

        print()
        print("=" * 68)
        print("  RESULT")
        print("=" * 68)
        if left and right:
            spread = abs(left - right) / ((left + right) / 2.0) * 100.0
            print(f"  left  : {left:.1f} ticks/rev")
            print(f"  right : {right:.1f} ticks/rev")
            print(f"  spread: {spread:.1f}%")
            print()
            print("  Put these in bot_firmware config.h:")
            print(f"      #define TICKS_PER_REV_LEFT    {left:.1f}f")
            print(f"      #define TICKS_PER_REV_RIGHT   {right:.1f}f")

            if spread > 3.0:
                print()
                if clean:
                    print("  The wheels disagree by more than 3%, but the noise check")
                    print("  was clean - so this is most likely two genuinely")
                    print("  different gearboxes. The per-wheel values above handle")
                    print("  that correctly. Compare each 'closest match' line: if")
                    print("  both land on a standard ratio, it is real.")
                else:
                    print("  The wheels disagree by more than 3% AND the noise check")
                    print("  failed. Fix the noise first - these numbers are not")
                    print("  trustworthy yet.")
        else:
            print("  Incomplete - see the messages above.")
        print()
        return 0

    except KeyboardInterrupt:
        print("\nCancelled.")
        return 1
    finally:
        executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
