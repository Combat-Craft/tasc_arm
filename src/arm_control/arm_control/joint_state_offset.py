#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateOffset(Node):
    def __init__(self):
        super().__init__("joint_state_offset")

        # Add these to the incoming joint positions to make RViz look "upright"
        self.offset_map = {
            "base_rotator_joint": -0.366519,
            "shoulder_joint": 0.436332,
            "elbow_joint": -0.872665,
            "wrist_joint": -1.13446,
            "end_joint": 1.46608,
            "gear_right_joint": 0.0,
        }
        
        self.sign_map = {
            "base_rotator_joint": -1.0,
            "shoulder_joint": -1.0,
            "elbow_joint": 1.0,
            "wrist_joint": 1.0,
            "end_joint": -1.0,
            "gear_right_joint": -1.0,
        }

        self.sub = self.create_subscription(JointState, "/joint_states", self.cb, 10)
        self.pub = self.create_publisher(JointState, "/joint_states_sim", 10)

        self.get_logger().info("Publishing /joint_states_sim = /joint_states + offsets")

    def cb(self, msg: JointState):
        out = JointState()
        out.header = msg.header
        out.name = list(msg.name)
        out.position = list(msg.position)
        out.velocity = list(msg.velocity) if msg.velocity else []
        out.effort = list(msg.effort) if msg.effort else []

        for i, n in enumerate(out.name):
            if i < len(out.position) and n in self.offset_map:
                sign = self.sign_map.get(n, 1.0)
                off = self.offset_map.get(n, 0.0)
                out.position[i] = sign * out.position[i] + off

        self.pub.publish(out)

def main():
    rclpy.init()
    node = JointStateOffset()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

