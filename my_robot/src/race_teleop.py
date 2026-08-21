#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class RaceTeleop(Node):

    def __init__(self):
        super().__init__('race_teleop_pro')

        self.sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ===== STATE =====
        self.target_linear = 0.0
        self.target_angular = 0.0

        self.current_linear = 0.0
        self.current_angular = 0.0

        # ===== PARAMETERS =====
        self.max_linear = 0.5
        self.max_angular = 1.2

        self.smoothing = 0.2
        self.deadzone = 0.1
        self.expo = 2.5

        # Timer → constant publishing
        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

    def joy_callback(self, msg):

        # ===== TRIGGERS =====
        lt = float((1 - msg.axes[2]) / 2)
        rt = float((1 - msg.axes[5]) / 2)

        # ===== JOYSTICK =====
        steer = float(msg.axes[0])

        # ===== DEADZONE =====
        if abs(steer) < self.deadzone:
            steer = 0.0

        # ===== EXPO CURVE (SAFE) =====
        if steer >= 0:
            steer = float(steer ** self.expo)
        else:
            steer = float(-((-steer) ** self.expo))  # preserve sign safely

        # ===== LINEAR =====
        self.target_linear = float((rt - lt) * self.max_linear)

        # ===== ANGULAR =====
        self.target_angular = float(steer * self.max_angular)

        # ===== SMART TURNING =====
        if abs(self.target_linear) > 0.05:
            self.target_angular = float(self.target_angular * 0.7)
        else:
            self.target_angular = float(self.target_angular * 1.2)

    def update(self):

        # ===== SMOOTHING =====
        self.current_linear += self.smoothing * (self.target_linear - self.current_linear)
        self.current_angular += self.smoothing * (self.target_angular - self.current_angular)

        # ===== DEADZONE CLEANUP =====
        if abs(self.current_linear) < 0.01:
            self.current_linear = 0.0

        if abs(self.current_angular) < 0.01:
            self.current_angular = 0.0

        # ===== FORCE FLOAT (CRITICAL) =====
        linear = float(self.current_linear)
        angular = float(self.current_angular)

        # ===== PUBLISH =====
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        self.pub.publish(twist)


def main():
    rclpy.init()
    node = RaceTeleop()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
