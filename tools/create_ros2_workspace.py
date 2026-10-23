"""Create a fresh, self-contained husky_vision workspace from current sources.

Run this with any Python >=3.10; no ROS, torch or OpenCV imports are needed.
The destination must not exist. Existing workspaces and model sources are read
only. After generation, the workspace does not depend on this repository.
"""

import argparse
import json
from pathlib import Path
import shlex
import shutil
import tempfile


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = "husky_vision"
SOURCE_PACKAGE = "husky_drivable_area"


def create_workspace(destination, model_source):
    destination = Path(destination).expanduser().absolute()
    if destination.is_symlink():
        raise FileExistsError(f"destination is an existing symlink: {destination}; choose a NEW workspace directory")
    destination = destination.resolve()
    model_source = Path(model_source).expanduser().resolve()
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(
            f"destination already exists: {destination}; choose a NEW workspace directory"
        )
    if destination.is_relative_to(model_source):
        raise ValueError("workspace must be outside the model source directory")
    for name in ("config.json", "preprocessor_config.json"):
        if not (model_source / name).is_file():
            raise ValueError(f"model directory is missing {name}: {model_source}")
    if not any((model_source / name).is_file() for name in (
        "model.safetensors", "pytorch_model.bin", "model.safetensors.index.json", "pytorch_model.bin.index.json",
    )):
        raise ValueError(f"no model weights found in {model_source}; use the extracted trained model folder")
    # A sibling staging directory prevents interrupted copies appearing complete.
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".husky-vision-", dir=destination.parent) as staging:
        workspace = Path(staging) / "workspace"
        package = workspace / "src" / PACKAGE
        package.mkdir(parents=True)
        for source, target in ((ROOT / "road_navigation", package / "road_navigation"),
                               (ROOT / SOURCE_PACKAGE, package / PACKAGE)):
            target.mkdir()
            for path in source.glob("*.py"):
                shutil.copy2(path, target / path.name)
        for name in ("setup.py", "setup.cfg", "package.xml", "launch/drivable_area.launch.py"):
            target = package / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(
                (ROOT / name).read_text(encoding="utf-8").replace(SOURCE_PACKAGE, PACKAGE),
                encoding="utf-8",
            )
        (package / "resource").mkdir()
        (package / "resource" / PACKAGE).touch()
        shutil.copy2(ROOT / "config.example.json", package / "config.example.json")
        (package / "requirements-ros2.txt").write_text(
            (ROOT / "requirements-ros2.txt").read_text(encoding="utf-8").replace(
                "docs/ros2-jazzy.md", "../../START_HERE.md",
            ), encoding="utf-8",
        )
        shutil.copytree(model_source, workspace / "models/husky-segformer")
        # Keep colcon from searching downloaded model Python/config files.
        (workspace / "models/COLCON_IGNORE").touch()
        (workspace / ".gitignore").write_text(
            ".venv/\nbuild/\ninstall/\nlog/\nmodels/\n__pycache__/\n*.py[cod]\n*.egg-info/\n",
            encoding="utf-8",
        )
        (workspace / "README.md").write_text(
            "# Husky vision — ROS 2 Jazzy\n\n"
            "Package: src/husky_vision\nModel: models/husky-segformer\n\n"
            "Install dependencies and build from this workspace using the guide in "
            "START_HERE.md. The source code and model here are independent copies.\n"
            "Topics: /camera/image_raw, /drivable_area/overlay, /drivable_area/mask, "
            "/drivable_area/navigable_mask, /drivable_area/reference_pixels, /drivable_area/status.\n"
            "References use processed-image pixels, not meters. No motor commands are published.\n",
            encoding="utf-8",
        )
        guide = (ROOT / "docs/ros2-fresh-start.md").read_text(encoding="utf-8")
        # START_HERE applies to the generated workspace, including custom paths.
        guide = guide[guide.index("## 2."):]
        guide = guide.replace('"$HOME/husky_vision_ws/models/husky-segformer"',
                              shlex.quote(str(destination / "models/husky-segformer")))
        guide = guide.replace("~/husky_vision_ws", shlex.quote(str(destination)))
        (workspace / "START_HERE.md").write_text(guide, encoding="utf-8")
        # Recheck before rename to avoid replacing a directory created meanwhile.
        if destination.exists() or destination.is_symlink():
            raise FileExistsError(f"destination appeared while copying: {destination}")
        workspace.rename(destination)
    return {
        "workspace": str(destination),
        "package": str(destination / "src" / PACKAGE),
        "model": str(destination / "models/husky-segformer"),
        "guide": str(destination / "START_HERE.md"),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True, help="NEW workspace path; must not exist")
    parser.add_argument("--model-source", type=Path, required=True, help="existing extracted trained SegFormer directory")
    args = parser.parse_args(argv)
    try:
        result = create_workspace(args.workspace, args.model_source)
    except (OSError, ValueError) as exc:
        parser.exit(1, f"Workspace creation failed: {exc}\n")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
