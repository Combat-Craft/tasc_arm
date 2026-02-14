#!/usr/bin/env python3
# logitech_teleop.py
#
# Subscribes: /joy  (sensor_msgs/Joy)
# Publishes:  /forward_position_controller/commands (std_msgs/Float64MultiArray)
#
# Controls (default mapping for Logitech Extreme 3D Pro):
#   Axis 0 (left/right) -> Joint 1
#   Axis 1 (up/down)    -> Joint 2 (inverted by default)
#   Axis 2 (twist)      -> Joint 3
#   Axis 3 (throttle)   -> Joint 4
#   Buttons 4/5         -> Joint 5 (-/+)
#   Buttons 2/3         -> Joint 6 (-/+)
#
# Notes:
# - This integrates velocity-like joystick inputs into position targets.
# - It adapts to 1..6 joints based on the "joints" parameter list length.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


class JoyTeleop(Node):
    def __init__(self):
        super().__init__("logitech_extreme3dpro_stepper_teleop")

        self.declare_parameter("topic", "/forward_position_controller/commands")
        self.declare_parameter("joints", ["motor_joint"])  # 1..6
        self.declare_parameter("limit_rad", 6.283185307179586)  # clamp target position
        self.declare_parameter("rate_hz", 50.0)

        # How fast each joint moves at full stick deflection (rad/s)
        self.declare_parameter("joint_speed_rad_s", [1.5, 1.5, 1.5, 1.5, 1.0, 1.0])

        # Axis indices for joints 1..4
        self.declare_parameter("axis_map", [0, 1, 2, 3])

        # Axis direction multipliers for joints 1..4 (use -1 to invert)
        self.declare_parameter("axis_sign", [1.0, -1.0, 1.0, 1.0])

        # Button mapping for joints 5 and 6: [neg_btn, pos_btn]
        self.declare_parameter("joint5_buttons", [4, 5])  # LB/RB style on some layouts
        self.declare_parameter("joint6_buttons", [2, 3])  # buttons on the stick base

        self.topic = self.get_parameter("topic").value
        self.joints = list(self.get_parameter("joints").value)
        self.limit = float(self.get_parameter("limit_rad").value)

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / max(1.0, self.rate_hz)

        speeds = list(self.get_parameter("joint_speed_rad_s").value)
        self.speeds = (speeds + [1.0] * 6)[:6]

        self.axis_map = list(self.get_parameter("axis_map").value)
        self.axis_sign = list(self.get_parameter("axis_sign").value)

        self.j5_btns = list(self.get_parameter("joint5_buttons").value)
        self.j6_btns = list(self.get_parameter("joint6_buttons").value)

        self.n = max(1, min(6, len(self.joints)))
        self.q = [0.0] * self.n

        self.last_joy = None

        self.pub = self.create_publisher(Float64MultiArray, self.topic, 10)
        self.sub = self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(f"Publishing to: {self.topic}")
        self.get_logger().info(f"Joints ({self.n}): {self.joints[:self.n]}")
        self.get_logger().info(f"rate_hz={self.rate_hz:.1f}, limit_rad={self.limit:.3f}")
        self.get_logger().info("Joy teleop ready.")

        self.publish()

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

    def on_joy(self, msg: Joy):
        self.last_joy = msg

    def axis_val(self, axes, idx):
        if idx < 0 or idx >= len(axes):
            return 0.0
        return float(axes[idx])

    def button_down(self, buttons, idx):
        if idx < 0 or idx >= len(buttons):
            return False
        return bool(buttons[idx])

    def tick(self):
        if self.last_joy is None:
            return

        axes = self.last_joy.axes
        buttons = self.last_joy.buttons

        # Joints 1..4 from axes (integrate into position)
        for j in range(min(self.n, 4)):
            a_idx = self.axis_map[j] if j < len(self.axis_map) else -1
            sign = self.axis_sign[j] if j < len(self.axis_sign) else 1.0
            a = self.axis_val(axes, a_idx) * sign
            self.q[j] = self.clamp(self.q[j] + a * self.speeds[j] * self.dt)

        # Joint 5 from buttons
        if self.n >= 5:
            neg = self.button_down(buttons, self.j5_btns[0]) if len(self.j5_btns) > 0 else False
            pos = self.button_down(buttons, self.j5_btns[1]) if len(self.j5_btns) > 1 else False
            direction = (1.0 if pos else 0.0) - (1.0 if neg else 0.0)
            self.q[4] = self.clamp(self.q[4] + direction * self.speeds[4] * self.dt)

        # Joint 6 from buttons
        if self.n >= 6:
            neg = self.button_down(buttons, self.j6_btns[0]) if len(self.j6_btns) > 0 else False
            pos = self.button_down(buttons, self.j6_btns[1]) if len(self.j6_btns) > 1 else False
            direction = (1.0 if pos else 0.0) - (1.0 if neg else 0.0)
            self.q[5] = self.clamp(self.q[5] + direction * self.speeds[5] * self.dt)

        self.publish()


def main():
    rclpy.init()
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
