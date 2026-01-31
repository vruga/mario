"""
Simple MuJoCo viewer for MARIO manipulator.

Usage:
    python -m scripts.viewer
    # or after install: mario-viewer
"""
import mujoco
import mujoco.viewer
from pathlib import Path


def get_model_path() -> Path:
    return Path(__file__).parent.parent / "manipulator.xml"


def main():
    model_path = get_model_path()
    print(f"Loading model: {model_path}")

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    print("\nModel Info:")
    print(f"  Joints: {model.njnt}")
    print(f"  Actuators: {model.nu}")
    print(f"  Bodies: {model.nbody}")

    joint_names = [model.joint(i).name for i in range(model.njnt)]
    print(f"  Joint names: {joint_names}")

    print("\nControls:")
    print("  - Use GUI sliders to control joints")
    print("  - Space: pause/resume")
    print("  - Backspace: reset")
    print("  - Esc: quit")

    # Use blocking launch() which works on macOS without mjpython
    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
