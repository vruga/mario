#!/usr/bin/env python3
import math
import sys
import numpy as np
import rerun as rr
import trimesh
import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import urllib.request
import json

# IP address of the ESP32 when running 5_servo_test_webserver firmware (HTTP path).
# Only used when NOT using micro-ROS (i.e. no micro-ROS agent running).
# For the normal micro-ROS flow (3_microros_rviz firmware + micro-ROS agent),
# joint states are forwarded via ROS2 /joint_states topic — no IP needed.
ESP32_IP = "192.168.1.100"  # <-- change this to your ESP32's IP if using webserver firmware

MESH_DIR = Path(get_package_share_directory("simulation_rviz")) / "meshes"

# Colors from URDF (RGBA 0-255)
GRAY   = [160, 160, 160, 255]
BLUE   = [202, 209, 238, 255]
SILVER = [198, 193, 188, 255]

# (entity_path, mesh_file, vertex_color, visual_xyz_offset_in_link_frame)
# visual_xyz_offset from each link's <visual><origin xyz=.../> in manipulator.urdf.
# All zero except claw_left which has xyz="0 -0.014 0.0075".
LINKS = [
    ("robot/base_link",  "base_link.STL",  GRAY,   [ 0.0,    0.0,    0.0    ]),
    ("robot/link_1",     "link_1.STL",     BLUE,   [ 0.0,    0.0,    0.0    ]),
    ("robot/link_2",     "link_2.STL",     BLUE,   [ 0.0,    0.0,    0.0    ]),
    ("robot/link_3",     "link_3.STL",     BLUE,   [ 0.0,    0.0,    0.0    ]),
    ("robot/claw_right", "claw_right.STL", SILVER, [ 0.0,    0.0,    0.0    ]),
    ("robot/claw_left",  "claw_left.STL",  SILVER, [ 0.0,   -0.014,  0.0075 ]),
]


def rot(axis, angle):
    ax, ay, az = np.array(axis, dtype=float)
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return np.array([
        [t*ax*ax + c,    t*ax*ay - s*az, t*ax*az + s*ay],
        [t*ax*ay + s*az, t*ay*ay + c,    t*ay*az - s*ax],
        [t*ax*az - s*ay, t*ay*az + s*ax, t*az*az + c   ],
    ])


def tf(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def fk(theta_base, theta_shoulder, theta_elbow, theta_claw1, theta_claw2):
    """
    Forward kinematics from manipulator.urdf joint chain.

    joint_1 (base_link → link_1):      Z axis,  [-0.0030196,  0.046937,   0.0635   ]
    joint_2 (link_1    → link_2):     -Y axis,  [ 0.00031511,-0.0095653,  0.034407 ]
    joint_3 (link_2    → link_3):     -Y axis,  [ 0.071531,   0.011279,  -0.0041348]
    joint_4 (link_3    → claw_right): -X axis,  [ 0.020925,  -0.013767,  -0.090331 ]
    joint_5 (link_3    → claw_left):  +X axis,  [ 0.0091023,  0.0182709, -0.090512 ]
    """
    T0 = np.eye(4)
    T1 = T0 @ tf(rot([0,  0, 1], theta_base),     [-0.0030196,  0.046937,   0.0635   ])
    T2 = T1 @ tf(rot([0, -1, 0], theta_shoulder),  [ 0.00031511,-0.0095653,  0.034407 ])
    T3 = T2 @ tf(rot([0, -1, 0], theta_elbow),     [ 0.071531,   0.011279,  -0.0041348])
    T4 = T3 @ tf(rot([-1, 0, 0], theta_claw1),     [ 0.020925,  -0.013767,  -0.090331 ])
    T5 = T3 @ tf(rot([ 1, 0, 0], theta_claw2),     [ 0.0091023,  0.0182709, -0.090512 ])
    return [T0, T1, T2, T3, T4, T5]


def load_meshes():
    meshes = {}
    for entity, stl, color, offset in LINKS:
        mesh = trimesh.load(str(MESH_DIR / stl), force="mesh")
        verts = np.array(mesh.vertices, dtype=np.float32) + np.array(offset, dtype=np.float32)
        faces = np.array(mesh.faces,    dtype=np.uint32)
        colors = np.tile(color, (len(verts), 1)).astype(np.uint8)
        meshes[entity] = rr.Mesh3D(
            vertex_positions=verts,
            triangle_indices=faces,
            vertex_colors=colors,
        )
    return meshes


class ArmVisualizer(Node):
    def __init__(self, meshes):
        super().__init__("arm_visualizer")
        self.pub = self.create_publisher(
            JointState, "joint_states",
            qos_profile=qos.qos_profile_parameter_events
        )
        self.meshes = meshes
        self.step = 0

        for entity, mesh in self.meshes.items():
            rr.log(entity, mesh, static=True)

    def send_to_esp32(self, theta_base, theta_shoulder, theta_elbow, theta_claw1, theta_claw2):
        """Send angles (degrees) directly to ESP32 via HTTP POST (5_servo_test_webserver firmware only).
        Skipped silently if ESP32 is unreachable; micro-ROS path uses /joint_states publish instead."""
        payload = json.dumps({
            "servo_a": theta_base,
            "servo_b": theta_shoulder,
            "servo_c": theta_elbow,
            "servo_d": theta_claw1,   # claw treated as single gripper command
        }).encode()
        try:
            req = urllib.request.Request(
                f"http://{ESP32_IP}/api/v1/servo",
                data=payload,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            urllib.request.urlopen(req, timeout=1)
        except Exception as e:
            self.get_logger().warn(f"ESP32 unreachable: {e}")

    def publish_and_visualize(self, theta_base, theta_shoulder, theta_elbow, theta_claw1, theta_claw2):
        rad = [a * math.pi / 180.0 for a in [theta_base, theta_shoulder, theta_elbow, theta_claw1, theta_claw2]]

        # micro-ROS path: publish to /joint_states → micro-ROS agent → ESP32 (3_microros_rviz)
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        msg.position = rad
        msg.velocity = []
        msg.effort = []
        self.pub.publish(msg)

        # Webserver fallback: also try HTTP direct to ESP32 (5_servo_test_webserver firmware), fails silently
        self.send_to_esp32(theta_base, theta_shoulder, theta_elbow, theta_claw1, theta_claw2)

        # Visualize in Rerun
        transforms = fk(*rad)
        rr.set_time("pose", sequence=self.step)
        for (entity, *_), T in zip(LINKS, transforms):
            rr.log(entity, rr.Transform3D(translation=T[:3, 3], mat3x3=T[:3, :3]))

        self.step += 1


def main():
    rclpy.init(args=sys.argv)
    rr.init("mario_arm", spawn=True)

    # ROS uses Z-up right-handed coordinates
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("robot", rr.Transform3D(translation=[0, 0, 0]), static=True)

    print("Loading meshes...")
    meshes = load_meshes()

    node = ArmVisualizer(meshes)
    node.get_logger().info("MARIO Robot Arm — Rerun Visualizer")
    print("Enter joint angles in degrees [0–180]. Ctrl+C to exit.\n")

    try:
        while True:
            try:
                base     = float(input("theta_base:     "))
                shoulder = float(input("theta_shoulder: "))
                elbow    = float(input("theta_elbow:    "))
                claw1    = float(input("theta_claw 1:   "))
                claw2    = float(input("theta_claw 2:   "))
            except (KeyboardInterrupt, EOFError):
                print("\nExiting.")
                break
            except ValueError:
                print("Invalid input — enter a number.")
                continue

            if not all(0.0 <= a <= 180.0 for a in [base, shoulder, elbow, claw1, claw2]):
                print("All angles must be in [0, 180].")
                continue

            node.publish_and_visualize(base, shoulder, elbow, claw1, claw2)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
