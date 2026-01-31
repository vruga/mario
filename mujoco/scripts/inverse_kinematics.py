"""
Inverse Kinematics using MuJoCo's built-in Jacobian computation.

MuJoCo provides mj_jacSite() to compute the Jacobian of a site.
We use this for iterative IK with damped least squares.

Usage:
    python -m scripts.inverse_kinematics
    # or after install: mario-ik
"""
import mujoco
import numpy as np
from pathlib import Path


def get_model_path() -> Path:
    return Path(__file__).parent.parent / "manipulator.xml"


def inverse_kinematics(
    model,
    data,
    target_pos: np.ndarray,
    max_iter: int = 100,
    tol: float = 1e-3,
    step_size: float = 0.5,
    damping: float = 0.01
) -> tuple[np.ndarray | None, float, int]:
    """
    Compute inverse kinematics using damped least squares (Levenberg-Marquardt).

    Args:
        model: MuJoCo model
        data: MuJoCo data
        target_pos: Target position (x, y, z)
        max_iter: Maximum iterations
        tol: Position tolerance
        step_size: Step size for updates
        damping: Damping factor for stability

    Returns:
        joint_angles: Solution angles (or None if failed)
        final_error: Final position error
        iterations: Number of iterations used
    """
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")

    # Start from middle of joint range (better initial guess)
    data.qpos[:3] = [np.pi/4, np.pi/4, np.pi/4]
    data.qpos[3:] = 0

    # Jacobian matrices (position and rotation)
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))

    for i in range(max_iter):
        # Compute current state
        mujoco.mj_forward(model, data)
        current_pos = data.site_xpos[site_id]

        # Compute error
        error = target_pos - current_pos
        error_norm = np.linalg.norm(error)

        if error_norm < tol:
            return data.qpos[:3].copy(), error_norm, i + 1

        # Compute Jacobian
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)

        # Use only first 3 columns (3 arm joints)
        J = jacp[:, :3]

        # Damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 * error
        JJT = J @ J.T + damping**2 * np.eye(3)
        dq = J.T @ np.linalg.solve(JJT, error)

        # Update joint positions
        data.qpos[:3] += step_size * dq

        # Clamp to joint limits
        for j in range(3):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{j+1}")
            jnt_range = model.jnt_range[joint_id]
            data.qpos[j] = np.clip(data.qpos[j], jnt_range[0], jnt_range[1])

    # Return best solution found
    mujoco.mj_forward(model, data)
    final_error = np.linalg.norm(target_pos - data.site_xpos[site_id])
    return data.qpos[:3].copy(), final_error, max_iter


def main():
    model_path = get_model_path()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    print("MARIO Inverse Kinematics (MuJoCo)")
    print("=" * 40)

    # First, show workspace by computing FK at different configurations
    print("\nWorkspace samples (FK at various configurations):")
    test_angles = [
        [0.0, np.pi/4, np.pi/4],
        [np.pi/2, np.pi/4, np.pi/4],
        [0.0, np.pi/2, np.pi/4],
    ]

    for angles in test_angles:
        data.qpos[:3] = angles
        mujoco.mj_forward(model, data)
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
        pos = data.site_xpos[site_id]
        print(f"  Angles (deg): [{np.degrees(angles[0]):.0f}, {np.degrees(angles[1]):.0f}, "
              f"{np.degrees(angles[2]):.0f}] -> Pos: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")

    # Test IK with known reachable targets (from FK above)
    print("\n" + "=" * 40)
    print("IK Test Cases:")

    # Use FK positions as test targets (guaranteed reachable)
    test_targets = [
        np.array([0.141, 0.049, 0.161]),   # From [0, 45, 45]
        np.array([-0.005, 0.191, 0.161]),  # From [90, 45, 45]
        np.array([0.054, 0.049, 0.244]),   # From [0, 90, 45]
    ]

    for target in test_targets:
        angles, error, iters = inverse_kinematics(model, data, target)

        print(f"\nTarget: [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
        if angles is not None:
            print(f"Solution (deg): [{np.degrees(angles[0]):.1f}, "
                  f"{np.degrees(angles[1]):.1f}, {np.degrees(angles[2]):.1f}]")
            print(f"Error: {error:.6f} m, Iterations: {iters}")

            # Verify with FK
            data.qpos[:3] = angles
            mujoco.mj_forward(model, data)
            site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
            actual_pos = data.site_xpos[site_id]
            print(f"Verification: [{actual_pos[0]:.3f}, {actual_pos[1]:.3f}, {actual_pos[2]:.3f}]")
        else:
            print("IK failed to converge")

    # Interactive mode
    print("\n" + "=" * 40)
    print("Interactive Mode (enter target x y z in meters, or 'q' to quit)")

    while True:
        try:
            user_input = input("\nTarget position (m) [x y z]: ").strip()
            if user_input.lower() == 'q':
                break

            target = np.array([float(x) for x in user_input.split()])
            if len(target) != 3:
                print("Please enter exactly 3 values")
                continue

            angles, error, iters = inverse_kinematics(model, data, target)

            if error < 0.01:
                print(f"Solution found in {iters} iterations")
                print(f"Joint angles (deg): [{np.degrees(angles[0]):.1f}, "
                      f"{np.degrees(angles[1]):.1f}, {np.degrees(angles[2]):.1f}]")
                print(f"Position error: {error*1000:.2f} mm")
            else:
                print(f"Target may be outside workspace (error: {error*1000:.1f} mm)")
                print(f"Best solution (deg): [{np.degrees(angles[0]):.1f}, "
                      f"{np.degrees(angles[1]):.1f}, {np.degrees(angles[2]):.1f}]")

        except ValueError:
            print("Invalid input. Enter 3 numbers separated by spaces.")
        except KeyboardInterrupt:
            break

    print("\nDone.")


if __name__ == "__main__":
    main()
