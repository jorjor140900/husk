"""Compare colour vetoes on an unpainted raw capture, offline without ROS/model.

Do not use a green overlay screenshot: its painted mask is not camera colour.
This reports colour-only changes, not a planned route or verified traversability.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

import cv2
import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from road_navigation.detector import DetectorConfig
from road_navigation.vegetation import green_vegetation_mask


ROOT = Path(__file__).resolve().parents[1]


def colour_mask(frame, config):
    settings = {key.removeprefix("vegetation_"): value for key, value in asdict(config).items()
                if key.startswith("vegetation_")}
    return green_vegetation_mask(frame, **settings)


def compare(frame, config):
    settings = asdict(config)
    settings["vegetation_support_radius_px"] = 0
    old = colour_mask(frame, DetectorConfig(**settings))
    new = colour_mask(frame, config)
    # Red: retained veto. Cyan: removed colour-only veto. Raw pixels elsewhere.
    image = frame.copy()
    image[new > 0] = (0, 0, 255)
    image[(old > 0) & (new == 0)] = (255, 255, 0)
    return old, new, image, dict(
        original_colour_veto_pixels=int(np.count_nonzero(old)),
        supported_colour_veto_pixels=int(np.count_nonzero(new)),
        removed_colour_only_pixels=int(np.count_nonzero((old > 0) & (new == 0))),
        added_pixels=int(np.count_nonzero((old == 0) & (new > 0))),
        support_radius_raw_pixels=config.vegetation_support_radius_px,
        support_fraction=config.vegetation_support_fraction,
        diagnostic_only=True, ready_for_control=False,
        note="Raw BGR colour-only comparison; no semantic model, map, TF or route validation.")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, required=True, help="raw camera image, before any overlay")
    parser.add_argument("--config", type=Path, default=ROOT/"config/outdoor-supported-green-preview.json")
    parser.add_argument("--output", type=Path, required=True, help="new output directory")
    args = parser.parse_args(argv)
    config = DetectorConfig(**json.loads(args.config.read_text()))
    frame = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if frame is None or min(frame.shape[:2]) < 16:
        parser.error("image must be a decodable raw capture at least 16x16")
    if not config.reject_green_vegetation or not config.vegetation_support_radius_px:
        parser.error("choose a config enabling green rejection and spatial support")
    old, new, image, report = compare(frame, config)
    args.output.mkdir(parents=True, exist_ok=False)
    for name, data in (("colour-before.png", old), ("colour-supported.png", new), ("changes.png", image)):
        if not cv2.imwrite(str(args.output/name), data):
            raise RuntimeError("could not write "+name)
    report["input_image"] = str(args.image.resolve())
    (args.output/"report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
