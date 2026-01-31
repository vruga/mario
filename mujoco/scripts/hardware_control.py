#!/usr/bin/env python3
"""
Interactive hardware control with MuJoCo visualization.

Control real hardware while seeing MuJoCo simulation mirror movements.

Usage:
    # Terminal 1: Start micro-ROS agent
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

    # Terminal 2: Run this controller
    python3 scripts/hardware_control.py

Controls:
    1/2: Joint 1 (base) -/+
    3/4: Joint 2 (shoulder) -/+
    5/6: Joint 3 (elbow) -/+
    7/8: Gripper open/close
    h: Home position
    q: Quit
"""
import numpy as np
from pathlib import Path
import sys
import tty
import termios
import select

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("ROS2 not available - simulation only mode")

import mujoco


def get_key(timeout=0.1):
    """Get keyboard input without blocking."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class HardwareController:
    def __init__(self):
        # Load MuJoCo model
        model_path = Path(__file__).parent.parent / "manipulator.xml"
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "end_effector"
        )

        # Joint angles (radians) - 5 joints
        self.joint_angles = np.array([0.0, np.pi/4, np.pi/4, 0.0, 0.0])
        self.increment = np.radians(5)  # 5 degree steps

        # ROS2 setup
        self.ros_node = None
        self.publisher = None
        if ROS_AVAILABLE:
            rclpy.init()
            self.ros_node = rclpy.create_node('mujoco_hardware_control')
            self.publisher = self.ros_node.create_publisher(
                Float64MultiArray,
                '/forward_position_controller/commands',
                10
            )
            print("ROS2 publisher ready: /forward_position_controller/commands")

    def update_simulation(self):
        """Update MuJoCo with current joint angles."""
        self.data.qpos[:5] = self.joint_angles
        mujoco.mj_forward(self.model, self.data)

    def get_ee_position(self):
        """Get end-effector position from FK."""
        return self.data.site_xpos[self.site_id].copy()

    def publish_to_hardware(self):
        """Send joint angles to real hardware via ROS2."""
        if self.publisher is not None:
            msg = Float64MultiArray()
            msg.data = self.joint_angles.tolist()
            self.publisher.publish(msg)

    def print_status(self):
        """Print current state."""
        ee = self.get_ee_position()
        angles_deg = np.degrees(self.joint_angles)
        print(f"\rJoints (deg): [{angles_deg[0]:6.1f}, {angles_deg[1]:6.1f}, "
              f"{angles_deg[2]:6.1f}, {angles_deg[3]:5.1f}, {angles_deg[4]:5.1f}] "
              f"| EE: [{ee[0]:.3f}, {ee[1]:.3f}, {ee[2]:.3f}]", end="    ")

    def run(self):
        """Main control loop."""
        print("\n" + "=" * 60)
        print("MARIO Hardware Controller (MuJoCo + ROS2)")
        print("=" * 60)
        print("\nControls:")
        print("  1/2: Joint 1 (base) -/+")
        print("  3/4: Joint 2 (shoulder) -/+")
        print("  5/6: Joint 3 (elbow) -/+")
        print("  7/8: Gripper close/open")
        print("  h: Home position")
        print("  q: Quit")
        print("=" * 60 + "\n")

        self.update_simulation()
        self.print_status()

        try:
            while True:
                key = get_key()
                if key is None:
                    continue

                changed = False

                if key == 'q':
                    print("\nQuitting...")
                    break
                elif key == 'h':
                    # Home position
                    self.joint_angles = np.array([0.0, np.pi/4, np.pi/4, 0.0, 0.0])
                    changed = True
                elif key == '1':
                    self.joint_angles[0] = max(0, self.joint_angles[0] - self.increment)
                    changed = True
                elif key == '2':
                    self.joint_angles[0] = min(np.pi, self.joint_angles[0] + self.increment)
                    changed = True
                elif key == '3':
                    self.joint_angles[1] = max(0, self.joint_angles[1] - self.increment)
                    changed = True
                elif key == '4':
                    self.joint_angles[1] = min(np.pi, self.joint_angles[1] + self.increment)
                    changed = True
                elif key == '5':
                    self.joint_angles[2] = max(0, self.joint_angles[2] - self.increment)
                    changed = True
                elif key == '6':
                    self.joint_angles[2] = min(np.pi, self.joint_angles[2] + self.increment)
                    changed = True
                elif key == '7':
                    # Close gripper
                    self.joint_angles[3] = min(0.8, self.joint_angles[3] + 0.1)
                    self.joint_angles[4] = min(0.8, self.joint_angles[4] + 0.1)
                    changed = True
                elif key == '8':
                    # Open gripper
                    self.joint_angles[3] = max(0, self.joint_angles[3] - 0.1)
                    self.joint_angles[4] = max(0, self.joint_angles[4] - 0.1)
                    changed = True

                if changed:
                    self.update_simulation()
                    self.publish_to_hardware()
                    self.print_status()

        finally:
            if self.ros_node is not None:
                self.ros_node.destroy_node()
                rclpy.shutdown()


def main():
    controller = HardwareController()
    controller.run()


if __name__ == '__main__':
    main()
