# MARIO MuJoCo Simulation

MuJoCo-based simulation for the MARIO manipulator. Provides fast physics simulation with built-in FK/IK computations.

## Quick Start - Hardware Control

```bash
# 1. Open Docker Desktop from Applications (first time only)
open /Applications/Docker.app

# 2. Run hardware control (starts micro-ROS agent automatically)
cd ~/twin/mario/mujoco
./run_hardware_control.sh
```

## Setup (Already Done)

ROS2 environment: `ros2_mario` (conda)
- ROS2 Humble (via Robostack)
- MuJoCo 3.4.0
- micro-ROS agent (via Docker)

```bash
# To manually activate environment:
conda activate ros2_mario
```

## Scripts

### 1. Viewer
Load and visualize the manipulator in MuJoCo's interactive viewer.

```bash
uv run python -m scripts.viewer
```

**Controls:**
- GUI sliders: control joints
- Space: pause/resume
- Backspace: reset
- Ctrl+Q: quit

### 2. Forward Kinematics
Compute end-effector position from joint angles using MuJoCo's built-in FK.

```bash
uv run python -m scripts.forward_kinematics
```

Example output:
```
Joint angles (deg): [45.0, 45.0, 45.0]
End-effector position (m): [0.0523, 0.0412, 0.0891]
```

### 3. Inverse Kinematics
Compute joint angles from target position using Jacobian-based IK.

```bash
uv run python -m scripts.inverse_kinematics
```

Example output:
```
Target: [0.05, 0.05, 0.10]
Solution (deg): [42.3, 38.7, 51.2]
Error: 0.000234 m
```

## Model Structure

```
mujoco/
├── manipulator.xml      # MuJoCo model (MJCF)
├── meshes/              # STL mesh files
│   ├── base_link.STL
│   ├── link_1.STL
│   ├── link_2.STL
│   ├── link_3.STL
│   ├── claw_right.STL
│   └── claw_left.STL
├── scripts/
│   ├── viewer.py              # Interactive viewer
│   ├── forward_kinematics.py  # FK computation
│   └── inverse_kinematics.py  # IK computation
├── pyproject.toml
└── README.md
```

## Joint Configuration

| Joint | Name | Type | Range (rad) | Description |
|-------|------|------|-------------|-------------|
| 1 | joint_1 | hinge | 0 to 3.14 | Base rotation (Z-axis) |
| 2 | joint_2 | hinge | 0 to 3.14 | Shoulder (Y-axis) |
| 3 | joint_3 | hinge | 0 to 3.14 | Elbow (Y-axis) |
| 4 | joint_4 | hinge | 0 to 0.8 | Claw right |
| 5 | joint_5 | hinge | 0 to 0.8 | Claw left |

## How MuJoCo FK/IK Works

**Forward Kinematics:**
- MuJoCo automatically computes FK when you call `mj_forward(model, data)`
- Set joint positions in `data.qpos[:]`
- Read end-effector position from `data.site_xpos[site_id]`

**Inverse Kinematics:**
- Use `mj_jacSite()` to compute the Jacobian at a site
- Apply damped least squares (Levenberg-Marquardt) for stable convergence
- Iterate until position error is below tolerance

---

## ROS2 Hardware Integration

Control real MARIO hardware via micro-ROS while visualizing in MuJoCo.

### Architecture

```
┌─────────────────┐     ROS2 Topics      ┌─────────────────┐
│  MuJoCo Sim     │ ──────────────────▶  │  micro-ROS      │
│  (this package) │  Float64MultiArray   │  Agent (UDP)    │
└─────────────────┘                      └────────┬────────┘
                                                  │ WiFi
                                                  ▼
                                         ┌─────────────────┐
                                         │  ESP32 + Servos │
                                         │  (firmware)     │
                                         └─────────────────┘
```

### Setup

```bash
# Terminal 1: Start micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Terminal 2: Interactive hardware control
source /opt/ros/humble/setup.bash
cd mujoco
python3 scripts/hardware_control.py
```

### Hardware Control Script

```bash
python3 scripts/hardware_control.py
```

**Controls:**
| Key | Action |
|-----|--------|
| 1/2 | Joint 1 (base) -/+ |
| 3/4 | Joint 2 (shoulder) -/+ |
| 5/6 | Joint 3 (elbow) -/+ |
| 7/8 | Gripper close/open |
| h   | Home position |
| q   | Quit |

### ROS2 Bridge Node

For custom integration:

```bash
python3 scripts/ros2_bridge.py
```

Publishes to: `/forward_position_controller/commands` (Float64MultiArray)
Subscribes to: `/mujoco/joint_commands` (Float64MultiArray)

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/forward_position_controller/commands` | Float64MultiArray | Joint commands to ESP32 (Gazebo mode) |
| `/joint_states` | JointState | Joint states (RViz mode) |
| `/mujoco/joint_commands` | Float64MultiArray | Input commands to bridge |
