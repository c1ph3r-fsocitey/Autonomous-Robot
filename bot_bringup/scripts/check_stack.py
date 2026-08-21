#!/usr/bin/env python3
"""Walk the whole control chain and report the FIRST thing that is broken.

    ros2 run bot_bringup check_stack.py

Run it with the robot brought up. It checks each link in order, because a
failure early on makes everything downstream look broken too:

    ESP32 <-> micro_ros_agent      /wheel_state is arriving
      -> bot_hardware              /wheel_cmd is being published
      -> controller_manager        both controllers are active
      -> diff_drive_controller     it is subscribed to its command topic
      -> twist_mux                 it is forwarding to that topic
      -> teleop                    something is producing Twists
      -> odometry + TF             odom -> base_footprint exists

Every failure comes with the specific command that fixes it.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

GREEN = "\033[1;32m"
RED = "\033[1;31m"
YELLOW = "\033[1;33m"
DIM = "\033[2m"
OFF = "\033[0m"

CMD_TOPIC = "/diff_drive_controller/cmd_vel_unstamped"


class StackChecker(Node):
    def __init__(self):
        super().__init__("check_stack")
        self.failures = 0
        self.warnings = 0

    # ---------------------------------------------------------------- helpers
    def ok(self, label, detail=""):
        print(f"  {GREEN}PASS{OFF}  {label}" + (f"  {DIM}{detail}{OFF}" if detail else ""))

    def fail(self, label, why, fix):
        self.failures += 1
        print(f"  {RED}FAIL{OFF}  {label}")
        print(f"        {why}")
        for line in fix.strip().splitlines():
            print(f"        {YELLOW}{line}{OFF}")

    def warn(self, label, why, fix=""):
        self.warnings += 1
        print(f"  {YELLOW}WARN{OFF}  {label}")
        print(f"        {why}")
        for line in fix.strip().splitlines():
            print(f"        {DIM}{line}{OFF}")

    def measure_rate(self, topic, msg_type, qos, seconds=2.0):
        """Count messages on a topic for a while. Returns messages per second."""
        count = [0]

        def cb(_):
            count[0] += 1

        sub = self.create_subscription(msg_type, topic, cb, qos)
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        self.destroy_subscription(sub)
        return count[0] / seconds

    def nodes(self):
        return [f"{ns.rstrip('/')}/{n}".replace("//", "/")
                for n, ns in self.get_node_names_and_namespaces()]

    # ----------------------------------------------------------------- checks
    def check_micro_ros(self):
        print(f"\n{DIM}1. ESP32 <-> micro_ros_agent{OFF}")

        if self.count_publishers("/wheel_state") == 0:
            self.fail(
                "/wheel_state has no publisher",
                "The ESP32 is not talking to a micro-ROS agent.",
                """
Is the agent running, and pointed at the right device?
    ls -l /dev/esp32 /dev/ttyACM* /dev/ttyUSB*
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200

If you started the robot with bringup.launch.py, pass the port to IT rather
than running a second agent - two agents on one port fight each other:
    ros2 launch bot_bringup bringup.launch.py esp32_port:=/dev/ttyACM0

The ESP32's LED must be SOLID. Blinking = still looking for the agent.
""")
            return False

        rate = self.measure_rate("/wheel_state", Float64MultiArray, BEST_EFFORT)
        if rate < 20:
            self.fail(
                f"/wheel_state is only {rate:.0f} Hz",
                "Expected ~50 Hz. The serial link is dropping data.",
                """
Check for a second process holding the port (a serial monitor, a stale agent):
    ls -l /proc/*/fd 2>/dev/null | grep -c ttyACM
Try a shorter USB cable, and confirm the motor supply ground is tied to the
ESP32 ground.
""")
            return False

        self.ok("/wheel_state", f"{rate:.0f} Hz")
        return True

    def check_hardware_interface(self):
        print(f"\n{DIM}2. bot_hardware (ros2_control -> ESP32){OFF}")

        if self.count_publishers("/wheel_cmd") == 0:
            self.fail(
                "/wheel_cmd has no publisher",
                "The bot_hardware plugin is not loaded or not activated.",
                """
    ros2 control list_hardware_components
Expect BotSystem in state 'active'. If it is 'unconfigured' the plugin failed
to load - look for a pluginlib error in the ros2_control_node output, and make
sure you re-sourced the workspace after building:
    source ~/ros2_ws/install/setup.bash
""")
            return False

        rate = self.measure_rate("/wheel_cmd", Float64MultiArray, BEST_EFFORT)
        if rate < 20:
            self.warn(
                f"/wheel_cmd is only {rate:.0f} Hz",
                "Expected ~50 Hz (the controller_manager update_rate).",
                "The Pi may be CPU starved. Close RViz and re-check.")
        else:
            self.ok("/wheel_cmd", f"{rate:.0f} Hz")
        return True

    def check_controllers(self):
        print(f"\n{DIM}3. controller_manager{OFF}")
        try:
            from controller_manager_msgs.srv import ListControllers
        except ImportError:
            self.warn("cannot import controller_manager_msgs", "skipping",
                      "sudo apt install ros-humble-controller-manager-msgs")
            return True

        client = self.create_client(ListControllers,
                                    "/controller_manager/list_controllers")
        if not client.wait_for_service(timeout_sec=5.0):
            self.fail(
                "/controller_manager is not running",
                "Nothing will move without it.",
                """
    ros2 launch bot_bringup bringup.launch.py esp32_port:=/dev/ttyACM0
Then look for errors in that terminal.
""")
            return False

        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            self.fail("list_controllers timed out", "controller_manager is wedged.",
                      "Restart bringup.launch.py")
            return False

        states = {c.name: c.state for c in future.result().controller}
        if not states:
            self.fail("no controllers loaded", "The spawners never ran.",
                      "Check the bringup terminal for spawner errors.")
            return False

        all_good = True
        for name in ("joint_state_broadcaster", "diff_drive_controller"):
            state = states.get(name)
            if state is None:
                self.fail(f"{name} is not loaded", "The spawner failed.",
                          f"ros2 run controller_manager spawner {name}")
                all_good = False
            elif state != "active":
                self.fail(f"{name} is '{state}', not 'active'", "It will ignore commands.",
                          f"ros2 control set_controller_state {name} active")
                all_good = False
            else:
                self.ok(name, state)
        return all_good

    def check_command_path(self):
        print(f"\n{DIM}4. command path into diff_drive_controller{OFF}")

        subs = self.count_subscribers(CMD_TOPIC)
        if subs == 0:
            self.fail(
                f"nothing is subscribed to {CMD_TOPIC}",
                "diff_drive_controller is not listening there. Almost always "
                "means use_stamped_vel is true, so it wants a TwistStamped on "
                "/diff_drive_controller/cmd_vel instead.",
                """
Check the setting:
    ros2 param get /diff_drive_controller use_stamped_vel
It must be false. It is set in bot_bringup/config/controllers.yaml.
""")
            return False
        self.ok(f"{CMD_TOPIC} has {subs} subscriber(s)")

        pubs = self.count_publishers(CMD_TOPIC)
        if pubs == 0:
            self.fail(
                f"nothing publishes to {CMD_TOPIC}",
                "This is the gap between teleop/Nav2 and the wheels.",
                """
Usually twist_mux is missing. Check:
    ros2 node list | grep twist_mux
    ros2 pkg list | grep twist_mux

If the package is not installed:
    sudo apt install -y ros-humble-twist-mux

Or skip the mux entirely and have teleop publish straight to the controller:
    ros2 launch bot_bringup bringup.launch.py use_twist_mux:=false \\
        esp32_port:=/dev/ttyACM0
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
""")
            return False
        self.ok(f"{CMD_TOPIC} has {pubs} publisher(s)")
        return True

    def check_teleop_sources(self):
        print(f"\n{DIM}5. teleop sources{OFF}")
        found = False
        for topic in ("/cmd_vel_joy", "/cmd_vel_key", "/cmd_vel", "/cmd_vel_smoothed"):
            n = self.count_publishers(topic)
            if n:
                self.ok(f"{topic}", f"{n} publisher(s)")
                found = True
        if not found:
            self.warn(
                "no teleop or navigation source is publishing",
                "Nothing is trying to drive the robot.",
                """
Keyboard (needs its own terminal, with focus):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -r /cmd_vel:=/cmd_vel_key
Joystick:
    ros2 launch bot_bringup teleop.launch.py
""")
        return True

    def check_odom_and_tf(self):
        print(f"\n{DIM}6. odometry and TF{OFF}")

        rate = self.measure_rate("/diff_drive_controller/odom", Odometry,
                                 QoSProfile(depth=10), seconds=1.5)
        if rate < 5:
            self.warn("/diff_drive_controller/odom is quiet",
                      f"{rate:.0f} Hz, expected ~50 Hz.",
                      "diff_drive_controller is active but not running.")
        else:
            self.ok("/diff_drive_controller/odom", f"{rate:.0f} Hz")

        try:
            import tf2_ros
            buf = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(buf, self)  # noqa: F841
            end = time.time() + 3.0
            while time.time() < end and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                if buf.can_transform("odom", "base_footprint",
                                     rclpy.time.Time()):
                    self.ok("TF odom -> base_footprint")
                    return True
            self.warn(
                "TF odom -> base_footprint is missing",
                "The EKF is not publishing it.",
                """
    ros2 node list | grep ekf
    ros2 topic hz /odometry/filtered
The EKF needs BOTH /diff_drive_controller/odom and /imu/data before it will
publish anything. Check /imu/data is alive:
    ros2 topic hz /imu/data
""")
        except ImportError:
            self.warn("tf2_ros not importable", "skipping the TF check")
        return True


def main():
    rclpy.init()
    node = StackChecker()

    print()
    print("=" * 70)
    print("  ROBOT STACK CHECK")
    print("=" * 70)

    try:
        # Give the graph a moment to be discovered, otherwise count_publishers
        # reports 0 for things that are perfectly healthy.
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)

        if node.check_micro_ros():
            if node.check_hardware_interface():
                if node.check_controllers():
                    if node.check_command_path():
                        node.check_teleop_sources()
                        node.check_odom_and_tf()

        print()
        print("=" * 70)
        if node.failures:
            print(f"  {RED}{node.failures} failure(s){OFF} - fix the FIRST one above, "
                  f"then re-run. Later checks are\n  suppressed because a broken "
                  f"link makes everything downstream look broken.")
        elif node.warnings:
            print(f"  {YELLOW}{node.warnings} warning(s){OFF}, no failures. "
                  f"The drive chain is intact.")
        else:
            print(f"  {GREEN}All good.{OFF} If it still will not move, the problem is "
                  f"mechanical or\n  a motor direction flag - see docs/CALIBRATION.md.")
        print("=" * 70)
        print()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 1 if node.failures else 0


if __name__ == "__main__":
    sys.exit(main())
