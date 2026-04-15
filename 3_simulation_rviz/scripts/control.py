#!/usr/bin/env python3
"""
Direct terminal control for MARIO arm servos.

Usage:
  python3 control.py                  # interactive prompt
  python3 control.py 0 90 45 0        # one-shot: set all 4 joints (degrees)

Joint order: base  shoulder  elbow  wrist
Range:       all joints accept degrees, converted to radians for /joint_states
"""

import sys
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

JOINT_NAMES = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]


class ArmController(Node):
    def __init__(self):
        super().__init__("arm_controller")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)

    def send(self, degrees: list[float]):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [math.radians(d) for d in degrees]
        self.pub.publish(msg)
        print(f"Sent: base={degrees[0]:.1f}°  shoulder={degrees[1]:.1f}°  elbow={degrees[2]:.1f}°  wrist={degrees[3]:.1f}°")


def parse_degrees(args: list[str]) -> list[float]:
    if len(args) != 4:
        print("Error: provide exactly 4 joint angles in degrees.")
        sys.exit(1)
    return [float(a) for a in args]


def main():
    rclpy.init()
    node = ArmController()

    # One-shot mode: angles passed as CLI args
    if len(sys.argv) > 1:
        degrees = parse_degrees(sys.argv[1:])
        node.send(degrees)
        # Give the publisher time to deliver before shutting down
        import time; time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()
        return

    # Interactive mode
    print("MARIO arm controller — enter 4 joint angles in degrees (Ctrl-C to quit)")
    print("Joints: base  shoulder  elbow  wrist")
    print()
    try:
        while rclpy.ok():
            try:
                line = input("angles> ").strip()
            except EOFError:
                break
            if not line:
                continue
            parts = line.split()
            if len(parts) != 4:
                print("  Need exactly 4 values, e.g.:  0 90 45 0")
                continue
            try:
                degrees = [float(p) for p in parts]
            except ValueError:
                print("  Invalid number.")
                continue
            node.send(degrees)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
