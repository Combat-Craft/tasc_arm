#!/usr/bin/env python3
# ps5_teleop.py
#
# Subscribes: /joy  (sensor_msgs/Joy)
# Publishes:  /forward_position_controller/commands (std_msgs/Float64MultiArray)
#
# Default mapping (common for DualSense via joy_node on Linux):
#   Left stick X (axis 0)  -> Joint 1
#   Left stick Y (axis 1)  -> Joint 2 (inverted)
#   Right stick X (axis 3) -> Joint 3
#   Right stick Y (axis 4) -> Joint 4 (inverted)
#   L2 / R2 (axes 2 and 5)  -> Joint 5 (R2 increases, L2 decreases)
#   L1 / R1 (buttons 4/5)   -> Joint 6 (R1 increases, L1 decreases)
#
# Notes:
# - This integrates joystick inputs into position targets (rad).
# - Adapts to 1..6 joints based on the "joints" parameter list length.
# - Axis indices can vary by driver. If it feels wrong, echo /joy and adjust params.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


class PS5Teleop(Node):
    def __init__(self):
        super().__init__("ps5_stepper_teleop")

        self.declare_parameter("topic", "/forward_position_controller/commands")
        self.declare_parameter("joints", ["motor_joint"])  # 1..6
        self.declare_parameter("limit_rad", 6.283185307179586)
        self.declare_parameter("rate_hz", 50.0)

        # rad/s at full deflection per joint
        self.declare_parameter("joint_speed_rad_s", [1.5, 1.5, 1.5, 1.5, 1.2, 1.2])

        # Stick axes for joints 1..4
        self.declare_parameter("axis_map", [0, 1, 3, 4])
        self.declare_parameter("axis_sign", [1.0, -1.0, 1.0, -1.0])

        # Trigger axes (L2, R2) for joint 5
        # Many drivers report triggers as [-1..+1] with -1 = released, +1 = fully pressed.
        self.declare_parameter("l2_axis", 2)
        self.declare_parameter("r2_axis", 5)

        # Shoulder buttons (L1, R1) for joint 6
        self.declare_parameter("l1_button", 4)
        self.declare_parameter("r1_button", 5)

        self.topic = self.get_parameter("topic").value
        self.joints = list(self.get_parameter("joints").value)
        self.limit = float(self.get_parameter("limit_rad").value)

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / max(1.0, self.rate_hz)

        speeds = list(self.get_parameter("joint_speed_rad_s").value)
        self.speeds = (speeds + [1.0] * 6)[:6]

        self.axis_map = list(self.get_parameter("axis_map").value)
        self.axis_sign = list(self.get_parameter("axis_sign").value)

        self.l2_axis = int(self.get_parameter("l2_axis").value)
        self.r2_axis = int(self.get_parameter("r2_axis").value)

        self.l1_button = int(self.get_parameter("l1_button").value)
        self.r1_button = int(self.get_parameter("r1_button").value)

        self.n = max(1, min(6, len(self.joints)))
        self.q = [0.0] * self.n

        self.last_joy = None

        self.pub = self.create_publisher(Float64MultiArray, self.topic, 10)
        self.sub = self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(f"Publishing to: {self.topic}")
        self.get_logger().info(f"Joints ({self.n}): {self.joints[:self.n]}")
        self.get_logger().info(f"rate_hz={self.rate_hz:.1f}, limit_rad={self.limit:.3f}")
        self.get_logger().info("PS5 teleop ready.")

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

    def trigger_to_01(self, raw):
        # Convert [-1..+1] to [0..1] if needed, otherwise clamp [0..1]
        if raw < -0.01:
            return (raw + 1.0) * 0.5
        if raw > 1.0:
            return 1.0
        if raw < 0.0:
            return 0.0
        return raw

    def tick(self):
        if self.last_joy is None:
            return

        axes = self.last_joy.axes
        buttons = self.last_joy.buttons

        # Joints 1..4 from sticks (integrate into position)
        for j in range(min(self.n, 4)):
            a_idx = self.axis_map[j] if j < len(self.axis_map) else -1
            sign = self.axis_sign[j] if j < len(self.axis_sign) else 1.0
            a = self.axis_val(axes, a_idx) * sign
            self.q[j] = self.clamp(self.q[j] + a * self.speeds[j] * self.dt)

        # Joint 5 from triggers (R2 increases, L2 decreases)
        if self.n >= 5:
            l2_raw = self.axis_val(axes, self.l2_axis)
            r2_raw = self.axis_val(axes, self.r2_axis)
            l2 = self.trigger_to_01(l2_raw)
            r2 = self.trigger_to_01(r2_raw)
            direction = r2 - l2
            self.q[4] = self.clamp(self.q[4] + direction * self.speeds[4] * self.dt)

        # Joint 6 from L1/R1 buttons (R1 increases, L1 decreases)
        if self.n >= 6:
            l1 = self.button_down(buttons, self.l1_button)
            r1 = self.button_down(buttons, self.r1_button)
            direction = (1.0 if r1 else 0.0) - (1.0 if l1 else 0.0)
            self.q[5] = self.clamp(self.q[5] + direction * self.speeds[5] * self.dt)

        self.publish()


def main():
    rclpy.init()
    node = PS5Teleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
