"""Make a clearly synthetic scene to try the offline runner without a dataset."""

import argparse
from pathlib import Path

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/demo-input"))
    args = parser.parse_args()
    rng = np.random.default_rng(11)
    h, w = 360, 640
    frame = np.full((h, w, 3), (40, 120, 40), np.uint8)
    frame[:108] = (180, 160, 140)
    road = np.zeros((h, w), np.uint8)
    cv2.fillPoly(road, [np.int32([[245, 108], [395, 108], [600, 359], [40, 359]])], 255)
    texture = rng.integers(-5, 6, (h, w), dtype=np.int16)
    floor = np.clip(135 + texture, 0, 255).astype(np.uint8)
    frame[road > 0] = np.repeat(floor[:, :, None], 3, axis=2)[road > 0]
    shadow = np.zeros((h, w), np.uint8)
    cv2.fillPoly(shadow, [np.int32([[180, 150], [480, 170], [530, 200], [140, 185]])], 255)
    shaded = (shadow > 0) & (road > 0)
    frame[shaded] = (frame[shaded] * 0.4).astype(np.uint8)
    obstacles = np.zeros((h, w), np.uint8)
    obstacles[208:270, 300:340] = 255
    frame[obstacles > 0] = (160, 35, 25)
    ground_truth = road.copy()
    ground_truth[obstacles > 0] = 0
    cv2.putText(frame, "SYNTHETIC DEMO - not real-world validation", (12, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
    args.output.mkdir(parents=True, exist_ok=False)
    for name, image in (("scene.png", frame), ("ground_truth.png", ground_truth), ("obstacles.png", obstacles)):
        if not cv2.imwrite(str(args.output / name), image):
            raise OSError(f"could not write {name}")
    print(args.output / "scene.png")


if __name__ == "__main__":
    main()
