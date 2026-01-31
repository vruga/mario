"""
Forward Kinematics using MuJoCo.

MuJoCo computes FK automatically when you call mj_forward().
The end-effector position is read from the site we defined.

Usage:
    python -m scripts.forward_kinematics
    # or after install: mario-fk
"""
import mujoco
import numpy as np
from pathlib import Path


def get_model_path() -> Path:
    return Path(__file__).parent.parent / "manipulator.xml"


def forward_kinematics(model, data, joint_angles: list[float]) -> tuple[np.ndarray, np.ndarray]:
    """
    Compute forward kinematics for given joint angles.

    Args:
        model: MuJoCo model
        data: MuJoCo data
        joint_angles: List of 3 joint angles [base, shoulder, elbow] in radians

    Returns:
        position: End-effector position (x, y, z)
        orientation: End-effector orientation matrix (3x3)
    """
    # Set joint positions (first 3 joints control arm position)
    data.qpos[:3] = joint_angles

    # Compute forward kinematics
    mujoco.mj_forward(model, data)

    # Get end-effector site data
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
    position = data.site_xpos[site_id].copy()
    orientation = data.site_xmat[site_id].reshape(3, 3).copy()

    return position, orientation


def main():
    model_path = get_model_path()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    print("MARIO Forward Kinematics (MuJoCo)")
    print("=" * 40)

    # Test configurations
    test_configs = [
        [0.0, 0.0, 0.0],                    # Home position
        [np.pi/4, np.pi/4, np.pi/4],        # 45 degrees each
        [np.pi/2, np.pi/3, np.pi/6],        # Mixed angles
        [0.0, np.pi/2, np.pi/2],            # Arm extended forward
    ]

    for angles in test_configs:
        pos, rot = forward_kinematics(model, data, angles)
        print(f"\nJoint angles (deg): [{np.degrees(angles[0]):.1f}, "
              f"{np.degrees(angles[1]):.1f}, {np.degrees(angles[2]):.1f}]")
        print(f"End-effector position (m): [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")

    # Interactive mode
    print("\n" + "=" * 40)
    print("Interactive Mode (enter 3 angles in degrees, or 'q' to quit)")

    while True:
        try:
            user_input = input("\nAngles (deg) [base shoulder elbow]: ").strip()
            if user_input.lower() == 'q':
                break

            angles_deg = [float(x) for x in user_input.split()]
            if len(angles_deg) != 3:
                print("Please enter exactly 3 angles")
                continue

            angles_rad = np.radians(angles_deg)
            pos, rot = forward_kinematics(model, data, angles_rad)

            print(f"End-effector position (m): [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
            print(f"Distance from origin: {np.linalg.norm(pos):.4f} m")

        except ValueError:
            print("Invalid input. Enter 3 numbers separated by spaces.")
        except KeyboardInterrupt:
            break

    print("\nDone.")


if __name__ == "__main__":
    main()
