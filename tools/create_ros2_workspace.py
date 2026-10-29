"""Create a fresh, self-contained husky_vision workspace from current sources.

Run this with any Python >=3.10; no ROS, torch or OpenCV imports are needed.
Creation requires a destination that does not exist and reads the model source
without changing it. --update-code updates ROS adapters, packaging and experiment
assets after backing them up. It preserves models, detector code and user camera
calibration. The generated workspace is self-contained.
"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shlex
import shutil
import tempfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = "husky_vision"
SOURCE_PACKAGE = "husky_drivable_area"


def ros_files():
    files = {name: ROOT / name for name in ("setup.py", "package.xml", "requirements-bridge.txt")}
    files.update({f"{PACKAGE}/{path.name}": path for path in (ROOT / SOURCE_PACKAGE).glob("*.py")
                  if path.name != "__init__.py"})
    files.update({str(path.relative_to(ROOT)): path for directory, pattern in (
        ("launch", "*.launch.py"), ("config", "*"), ("docs", "ros2-metric-path.md"), ("docs", "ros2-control.md"), ("docs", "robot-field-kit.md"), ("docs", "ros2-mount-fit.md"), ("docs", "ros2-charuco.md"), ("docs", "ros2-goal-navigation.md"), ("docs", "ros2-outdoor-grass.md"), ("docs", "ros2-image-goal.md"),
    ) for path in (ROOT / directory).glob(pattern) if path.is_file()})
    for path in (ROOT / "husky_link_core").glob("*.py"):
        files[str(path.relative_to(ROOT))] = path
        files["ros1/husky_vision_bridge/src/husky_link_core/"+path.name] = path
    for path in (ROOT / "ros1").rglob("*"):
        if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc":
            files[str(path.relative_to(ROOT))] = path
    return files


def renamed_content(source):
    if source.suffix == ".pdf":
        return source.read_bytes()
    return source.read_text(encoding="utf-8").replace(SOURCE_PACKAGE, PACKAGE).encode("utf-8")


def update_navigation_plugin(destination):
    """Install the companion C++ layer sources, backing up local edits first."""
    destination = Path(destination).expanduser().resolve()
    source = ROOT / "nav2_plugins/husky_vision_nav2"
    package = destination / "src/husky_vision_nav2"
    if package.exists():
        manifest = package / "package.xml"
        if not manifest.is_file() or ET.parse(manifest).findtext("name") != "husky_vision_nav2":
            raise ValueError("refusing to replace unrelated navigation package: "+str(package))
    contents = {}
    for path in source.rglob("*"):
        if not path.is_file() or "__pycache__" in path.parts:
            continue
        target = package / path.relative_to(source)
        if (path.is_symlink() or target.is_symlink() or
                any(parent.is_symlink() for parent in target.parents) or
                (target.exists() and not target.is_file())):
            raise ValueError("expected regular navigation source: "+str(target))
        contents[str(path.relative_to(source))] = path.read_bytes()
    backup = destination / ".source-backups" / datetime.now(timezone.utc).strftime("nav2-%Y%m%dT%H%M%S%fZ")
    backup.mkdir(parents=True, exist_ok=False)
    (backup.parent / "COLCON_IGNORE").touch()
    for name in contents:
        if (package/name).exists():
            saved = backup/name
            saved.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(package/name,saved)
    for name, data in contents.items():
        (package/name).parent.mkdir(parents=True,exist_ok=True)
        (package/name).write_bytes(data)
    return {"package":str(package),"source_backup":str(backup)}


def update_workspace_code(destination, *, update_visualization=False, with_navigation=False, update_detector=False):
    """Back up ROS files before updating; preserve detector/model/user calibration."""
    destination = Path(destination).expanduser().resolve()
    package = destination / "src" / PACKAGE
    manifest = package / "package.xml"
    if not manifest.is_file() or ET.parse(manifest).findtext("name") != PACKAGE:
        raise ValueError(f"expected an existing {PACKAGE} package at {package}")
    updates = ros_files()
    # The webcam adapter's diagnostics require the matching capture worker.
    updates["road_navigation/capture.py"] = ROOT / "road_navigation/capture.py"
    if update_visualization:
        updates["road_navigation/visualization.py"] = ROOT / "road_navigation/visualization.py"
    if update_detector:
        for name in ("detector.py", "vegetation.py", "cli.py"):
            updates["road_navigation/"+name] = ROOT / "road_navigation" / name
    required_existing = {f"{PACKAGE}/nodes.py", f"{PACKAGE}/transport.py",
                         "launch/drivable_area.launch.py", "setup.py", "package.xml"}
    contents = {}
    for relative, source in updates.items():
        target = package / relative
        if ((relative in required_existing and not target.is_file()) or
                (target.exists() and not target.is_file()) or target.is_symlink() or
                any(parent.is_symlink() for parent in target.parents if parent != package.parent) or
                not target.resolve().is_relative_to(package.resolve())):
            raise ValueError(f"expected a regular package source file: {target}")
        contents[relative] = renamed_content(source)
    backups = destination / ".source-backups"
    if backups.is_symlink():
        raise ValueError(f"backup directory must not be a symlink: {backups}")
    backups.mkdir(exist_ok=True)
    (backups / "COLCON_IGNORE").touch()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    backup = backups / stamp
    # Back up every target before changing any source, including local edits.
    created = []
    for relative in contents:
        if (package / relative).exists():
            saved = backup / relative
            saved.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(package / relative, saved)
        else:
            created.append(relative)
    backup.mkdir(parents=True, exist_ok=True)
    (backup / "created_files.json").write_text(json.dumps(created, indent=2), encoding="utf-8")
    written = []
    try:
        for relative, content in contents.items():
            written.append(relative)
            (package / relative).parent.mkdir(parents=True, exist_ok=True)
            (package / relative).write_bytes(content)
    except OSError:
        for relative in written:
            if relative in created:
                (package / relative).unlink(missing_ok=True)
            else:
                shutil.copy2(backup / relative, package / relative)
        raise
    result = {"workspace": str(destination), "updated_files": list(contents), "source_backup": str(backup)}
    if with_navigation:
        result["navigation_plugin"] = update_navigation_plugin(destination)
    return result


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
        for name, source in {**ros_files(), "setup.cfg": ROOT / "setup.cfg"}.items():
            target = package / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(renamed_content(source))
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
            "Pixel references use processed-image pixels. To add a calibrated metric projection, "
            "follow src/husky_vision/docs/ros2-metric-path.md. Perception does not publish motor commands.\n"
            "For an isolated controller simulation, follow src/husky_vision/docs/ros2-control.md.\n",
            encoding="utf-8",
        )
        guide = (ROOT / "docs/ros2-fresh-start.md").read_text(encoding="utf-8")
        # START_HERE applies to the generated workspace, including custom paths.
        guide = guide[guide.index("## 2."):]
        guide = guide.replace("](ros2-metric-path.md)", "](src/husky_vision/docs/ros2-metric-path.md)")
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
    parser.add_argument("--workspace", type=Path, required=True, help="workspace path; must be NEW unless --update-code is used")
    parser.add_argument("--model-source", type=Path, help="existing extracted trained SegFormer directory; required when creating")
    parser.add_argument("--update-code", action="store_true", help="update ROS adapters, packaging and experiment assets after backing up existing files")
    parser.add_argument("--update-visualization", action="store_true", help="also back up and update visualization.py; requires --update-code")
    parser.add_argument("--update-detector", action="store_true", help="also back up and update detector/vegetation veto code; requires --update-code")
    parser.add_argument("--with-navigation", action="store_true", help="also install/back up companion Nav2 C++ layer sources; requires --update-code")
    args = parser.parse_args(argv)
    if args.update_visualization and not args.update_code:
        parser.error("--update-visualization requires --update-code")
    if args.update_detector and not args.update_code:
        parser.error("--update-detector requires --update-code")
    if args.with_navigation and not args.update_code:
        parser.error("--with-navigation requires --update-code")
    if not args.update_code and args.model_source is None:
        parser.error("--model-source is required when creating a workspace")
    if args.update_code and args.model_source is not None:
        parser.error("--update-code does not use --model-source")
    try:
        result = update_workspace_code(args.workspace, update_visualization=args.update_visualization,
            with_navigation=args.with_navigation, update_detector=args.update_detector) if args.update_code else create_workspace(args.workspace, args.model_source)
    except (OSError, ValueError, ET.ParseError) as exc:
        parser.exit(1, f"Workspace creation failed: {exc}\n")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
