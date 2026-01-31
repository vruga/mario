"""
Interactive FK visualization - move joints and see end-effector position.

Usage:
    mjpython -m scripts.fk_interactive
"""
import mujoco
import mujoco.viewer
import numpy as np
from pathlib import Path
import time


def get_model_path() -> Path:
    return Path(__file__).parent.parent / "manipulator.xml"


def main():
    model_path = get_model_path()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")

    # Joint targets (will animate through these)
    targets = [
        [0.0, 0.0, 0.0],
        [0.0, np.pi/4, np.pi/4],
        [np.pi/2, np.pi/4, np.pi/4],
        [np.pi/2, np.pi/2, np.pi/4],
        [np.pi, np.pi/3, np.pi/3],
        [0.0, np.pi/2, np.pi/2],
        [0.0, 0.0, 0.0],
    ]

    current_target_idx = 0
    last_switch_time = time.time()
    switch_interval = 2.0  # seconds between targets

    print("Interactive FK Visualization")
    print("=" * 40)
    print("Watch joints move and end-effector position update")
    print("Joint sliders in right panel also work")
    print("=" * 40)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # Animate through targets
            current_time = time.time()
            if current_time - last_switch_time > switch_interval:
                current_target_idx = (current_target_idx + 1) % len(targets)
                last_switch_time = current_time
                target = targets[current_target_idx]
                print(f"\nTarget angles (deg): [{np.degrees(target[0]):.0f}, "
                      f"{np.degrees(target[1]):.0f}, {np.degrees(target[2]):.0f}]")

            # Smoothly interpolate to target
            target = targets[current_target_idx]
            for i in range(3):
                data.ctrl[i] = target[i]

            # Step simulation
            mujoco.mj_step(model, data)

            # Get FK result
            ee_pos = data.site_xpos[site_id]

            # Print FK result periodically
            if int(current_time * 2) % 2 == 0 and abs(current_time - int(current_time)) < 0.01:
                print(f"EE pos: [{ee_pos[0]:.4f}, {ee_pos[1]:.4f}, {ee_pos[2]:.4f}]")

            viewer.sync()


if __name__ == "__main__":
    main()
