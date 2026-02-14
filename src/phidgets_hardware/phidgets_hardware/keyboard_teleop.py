#!/usr/bin/env python3
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

LIMIT = 6.283185307179586  # default clamp: 2*pi rad

HELP = """
Teleop keys (press, no Enter):
  Joint 1: q / a
  Joint 2: w / s
  Joint 3: e / d
  Joint 4: r / f
  Joint 5: t / g
  Joint 6: y / h

  Step size: [ decrease ] increase
  Reset all: 0
  Quit: x
"""

KEYS = {
    'q': (0, +1), 'a': (0, -1),
    'w': (1, +1), 's': (1, -1),
    'e': (2, +1), 'd': (2, -1),
    'r': (3, +1), 'f': (3, -1),
    't': (4, +1), 'g': (4, -1),
    'y': (5, +1), 'h': (5, -1),
}

def get_key(timeout=0.1):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return None

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("phidgets_stepper_keyboard_teleop")

        # Params to keep it flexible
        self.declare_parameter("topic", "/forward_position_controller/commands")
        self.declare_parameter("joints", ["motor_joint"])  # set to 1..6 joint names
        self.declare_parameter("limit_rad", LIMIT)
        self.declare_parameter("step_rad", 0.30)

        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.joints = list(self.get_parameter("joints").get_parameter_value().string_array_value)
        self.limit = float(self.get_parameter("limit_rad").value)
        self.step = float(self.get_parameter("step_rad").value)

        self.n = max(1, min(6, len(self.joints)))
        self.q = [0.0] * self.n

        self.pub = self.create_publisher(Float64MultiArray, self.topic, 10)

        self.get_logger().info(HELP)
        self.get_logger().info(f"Publishing to: {self.topic}")
        self.get_logger().info(f"Joints ({self.n}): {self.joints[:self.n]}")
        self.get_logger().info(f"limit={self.limit:.3f} rad, step={self.step:.3f} rad")

    def clamp(self, v: float) -> float:
        if v > self.limit:
            return self.limit
        if v < -self.limit:
            return -self.limit
        return v

    def publish(self):
        msg = Float64MultiArray()
        msg.data = list(self.q)
        self.pub.publish(msg)

    def bump(self, idx: int, delta: float):
        if idx >= self.n:
            return
        self.q[idx] = self.clamp(self.q[idx] + delta)
        self.publish()

def main():
    rclpy.init()
    node = KeyboardTeleop()

    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        node.get_logger().info("Focus this terminal and press keys (no Enter).")
        node.publish()

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            k = get_key(0.1)
            if not k:
                continue

            if k == "x":
                node.get_logger().info("Exiting teleop.")
                break

            if k == "0":
                node.q = [0.0] * node.n
                node.publish()
                node.get_logger().info("Reset all joints to 0")
                continue

            if k == "[":
                node.step = max(0.01, node.step - 0.01)
                node.get_logger().info(f"step = {node.step:.2f} rad")
                continue

            if k == "]":
                node.step = min(0.50, node.step + 0.01)
                node.get_logger().info(f"step = {node.step:.2f} rad")
                continue

            if k in KEYS:
                idx, sign = KEYS[k]
                node.bump(idx, sign * node.step)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
