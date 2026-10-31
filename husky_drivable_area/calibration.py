"""Import only a validated monocular camera YAML from ROS calibration output."""

import argparse
from pathlib import Path
import tarfile

import yaml

from .ground_geometry import CameraCalibration


def import_calibration(archive, output):
    # Do not extract archive paths or the bundled captured images.
    with tarfile.open(Path(archive).expanduser(), "r:gz") as source:
        candidates = [m for m in source.getmembers() if m.name == "ost.yaml"]
        if len(candidates) != 1 or not candidates[0].isfile() or candidates[0].size > 1024 * 1024:
            raise ValueError("archive must contain one regular monocular ost.yaml file <= 1 MiB")
        content = source.extractfile(candidates[0]).read().decode("utf-8")
    camera = CameraCalibration.from_mapping(yaml.safe_load(content))
    output = Path(output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation also refuses existing symlinks. Keep prior calibration.
    with output.open("x", encoding="utf-8") as target:
        target.write(content)
    return camera


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, default=Path("/tmp/calibrationdata.tar.gz"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        camera = import_calibration(args.archive, args.output)
    except (OSError, ValueError, tarfile.TarError, yaml.YAMLError) as exc:
        parser.exit(1, f"Calibration not imported: {exc}\n")
    print(f"Saved {args.output.expanduser()}: {camera.width}x{camera.height}, pinhole calibration.")
    print("Keep camera capture resolution, focus and zoom unchanged after calibration.")
    return 0
