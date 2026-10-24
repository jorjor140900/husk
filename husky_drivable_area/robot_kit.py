"""Create a self-contained robot-side catkin package and one measured profile."""
import argparse
import json
from pathlib import Path
import shutil
import tempfile


def assets_root():
    root = Path(__file__).resolve().parents[1]
    if (root / "ros1/husky_vision_bridge/package.xml").is_file():
        return root
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("husky_drivable_area"))


def create_robot_kit(workspace, source_root=None):
    workspace = Path(workspace).expanduser().resolve()
    if not (workspace / "src/husky_vision/package.xml").is_file():
        raise ValueError("expected the existing husky_vision_ws with src/husky_vision")
    output = workspace / "robot"
    if output.exists() or output.is_symlink():
        raise FileExistsError("robot kit already exists; keep its measured profile: "+str(output))
    root = Path(source_root) if source_root is not None else assets_root()
    profile = json.loads((root / "config/robot.example.json").read_text())
    calibration = workspace / "camera_calibration/camera-v2.yaml"
    if not calibration.exists() and (workspace / "camera_calibration/camera.yaml").exists():
        calibration = workspace / "camera_calibration/camera.yaml"
    profile["camera"].update({"model_path": str(workspace / "models/husky-segformer"),
                              "calibration_path": str(calibration), "mount_path": str(output / "robot-mount.json")})
    mount = {"height_m": None, "pitch_down_deg": None, "roll_deg": 0.0, "yaw_deg": 0.0,
             "camera_x_m": None, "camera_y_m": None, "ground_frame": "husky_vision_ground",
             "camera_frame": "husky_vision_camera_optical_frame"}
    with tempfile.TemporaryDirectory(prefix=".robot-kit-", dir=workspace) as temporary:
        staged = Path(temporary) / "robot"
        staged.mkdir()
        (staged / "COLCON_IGNORE").touch()
        (staged / "robot.json").write_text(json.dumps(profile, indent=2)+"\n")
        (staged / "robot-mount.json").write_text(json.dumps(mount, indent=2)+"\n")
        shutil.copytree(root / "ros1/husky_vision_bridge", staged / "ros1_catkin/src/husky_vision_bridge",
                        symlinks=False, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        for script in (staged / "ros1_catkin/src/husky_vision_bridge/scripts").glob("*.py"):
            script.chmod(script.stat().st_mode | 0o111)
        shutil.copy2(root / "docs/robot-field-kit.md", staged / "README.md")
        staged.rename(output)
    return {"kit": str(output), "profile": str(output / "robot.json"), "mount": str(output / "robot-mount.json"),
            "ros1_workspace": str(output / "ros1_catkin"), "guide": str(output / "README.md"),
            "mode": "observe; measured robot values intentionally pending"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", required=True)
    args = parser.parse_args()
    print(json.dumps(create_robot_kit(args.workspace), indent=2))
