#!/usr/bin/env python3
"""
ROS2 Bridge for MuJoCo MARIO simulation.

Publishes joint angles to hardware via micro-ROS.

Usage:
    # Terminal 1: Start micro-ROS agent
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

    # Terminal 2: Run this bridge
    ros2 run mario_mujoco ros2_bridge
    # or directly:
    python3 scripts/ros2_bridge.py
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

try:
    import mujoco
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False
    print("Warning: mujoco not installed, running in mock mode")

from pathlib import Path


class MuJoCoROS2Bridge(Node):
    """Bridge between MuJoCo simulation and ROS2/micro-ROS hardware."""

    def __init__(self):
        super().__init__('mujoco_ros2_bridge')

        # Parameters
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('topic_mode', 'gazebo')  # 'gazebo' or 'rviz'

        rate = self.get_parameter('publish_rate').value
        self.topic_mode = self.get_parameter('topic_mode').value

        # Load MuJoCo model
        model_path = Path(__file__).parent.parent / "manipulator.xml"
        if MUJOCO_AVAILABLE and model_path.exists():
            self.model = mujoco.MjModel.from_xml_path(str(model_path))
            self.data = mujoco.MjData(self.model)
            self.site_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SITE, "end_effector"
            )
            self.get_logger().info(f"Loaded MuJoCo model: {model_path}")
        else:
            self.model = None
            self.data = None
            self.get_logger().warn("Running without MuJoCo model")

        # Current joint angles (radians)
        self.joint_angles = np.zeros(5)

        # Publishers - choose based on mode
        if self.topic_mode == 'gazebo':
            # For firmware/4_microros_gazebo (Float64MultiArray)
            self.cmd_pub = self.create_publisher(
                Float64MultiArray,
                '/forward_position_controller/commands',
                10
            )
            self.get_logger().info("Publishing to /forward_position_controller/commands")
        else:
            # For firmware/3_microros_rviz (JointState)
            self.cmd_pub = self.create_publisher(
                JointState,
                '/joint_states',
                10
            )
            self.get_logger().info("Publishing to /joint_states")

        # Subscriber for external commands (e.g., from teleop)
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/mujoco/joint_commands',
            self.command_callback,
            10
        )

        # Timer for publishing
        self.create_timer(1.0 / rate, self.publish_joints)

        self.get_logger().info(
            f"MuJoCo ROS2 Bridge started (mode: {self.topic_mode}, rate: {rate}Hz)"
        )
        self.get_logger().info("Subscribe to /mujoco/joint_commands to set angles")

    def command_callback(self, msg: Float64MultiArray):
        """Receive joint commands and update MuJoCo state."""
        if len(msg.data) >= 3:
            self.joint_angles[:len(msg.data)] = msg.data[:5]

            # Update MuJoCo simulation
            if self.model is not None:
                self.data.qpos[:len(msg.data)] = msg.data[:5]
                mujoco.mj_forward(self.model, self.data)

                # Log FK result
                ee_pos = self.data.site_xpos[self.site_id]
                self.get_logger().info(
                    f"Joints (deg): {np.degrees(self.joint_angles[:3]).round(1).tolist()} "
                    f"-> EE: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]"
                )

    def publish_joints(self):
        """Publish current joint angles to hardware."""
        if self.topic_mode == 'gazebo':
            msg = Float64MultiArray()
            msg.data = self.joint_angles.tolist()
            self.cmd_pub.publish(msg)
        else:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
            msg.position = self.joint_angles.tolist()
            self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoROS2Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
