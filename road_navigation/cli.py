"""Offline image/video/webcam runner. ROS and model weights are not required."""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np

from .detector import DetectorConfig, RoadDetector
from .visualization import draw_result


def _write_json(path: Path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _write_png(path: Path, image: np.ndarray):
    if not cv2.imwrite(str(path), image):
        raise OSError(f"could not write {path}")


def _new_output(path: Path, config: DetectorConfig):
    # Never overwrite a previous experiment or user file.
    path.mkdir(parents=True, exist_ok=False)
    _write_json(path / "config.json", asdict(config))


def _show(image: np.ndarray) -> bool:
    cv2.imshow("Road detection", image)
    return cv2.waitKey(1) & 0xFF in (ord("q"), 27)


def _run_image(args, detector: RoadDetector, output: Path) -> dict:
    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"could not read image: {args.image}")
    probability = np.load(args.probability, allow_pickle=False) if args.probability else None
    blocked = None
    if args.blocked_mask:
        blocked = cv2.imread(str(args.blocked_mask), cv2.IMREAD_GRAYSCALE)
        if blocked is None:
            raise ValueError(f"could not read blocked mask: {args.blocked_mask}")
    result = detector.process(image, drivable_probability=probability, blocked_mask=blocked)
    _new_output(output, detector.config)
    overlay = draw_result(result)
    _write_png(output / "overlay.png", overlay)
    _write_png(output / "raw_mask.png", result.raw_mask)
    _write_png(output / "mask.png", result.road_mask)
    _write_png(output / "navigable_mask.png", result.navigable_mask)
    np.save(output / "probability.npy", result.probability, allow_pickle=False)
    _write_json(output / "result.json", result.metadata())
    if args.show:
        cv2.imshow("Road detection", overlay)
        cv2.waitKey(0)
    return {"frames": 1, "valid_image_paths": int(result.valid_image_path), "mean_processing_ms": result.processing_ms}


def _even_frame(image: np.ndarray) -> np.ndarray:
    # MJPEG writers require even dimensions. Coordinates remain at top-left.
    h, w = image.shape[:2]
    return cv2.copyMakeBorder(image, 0, h % 2, 0, w % 2, cv2.BORDER_CONSTANT)


def _run_video(args, detector: RoadDetector, output: Path) -> dict:
    source = args.camera if args.camera is not None else str(args.video)
    capture = cv2.VideoCapture(source)
    writer = None
    processed = valid = 0
    durations = []
    stopped_by_user = False
    try:
        if not capture.isOpened():
            raise ValueError(f"could not open source: {source}")
        ok, frame = capture.read()
        if not ok:
            raise ValueError(f"source contains no decodable frames: {source}")
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        if not np.isfinite(fps) or fps <= 0:
            fps = 30.0
        expected_frames = capture.get(cv2.CAP_PROP_FRAME_COUNT) if args.video else 0
        _new_output(output, detector.config)
        if args.save_masks:
            (output / "masks").mkdir()
        with (output / "frames.jsonl").open("w", encoding="utf-8") as metadata:
            while ok:
                result = detector.process(frame)
                overlay = _even_frame(draw_result(result))
                if writer is None:
                    size = (overlay.shape[1], overlay.shape[0])
                    writer = cv2.VideoWriter(str(output / "overlay.avi"), cv2.VideoWriter_fourcc(*"MJPG"), fps, size)
                    if not writer.isOpened():
                        raise OSError("could not open MJPEG output writer")
                    output_shape = overlay.shape[:2]
                elif overlay.shape[:2] != output_shape:
                    raise ValueError("frame dimensions changed; split the source into separate clips")
                writer.write(overlay)
                row = result.metadata()
                row["frame_index"] = processed
                row["capture_position_ms"] = float(capture.get(cv2.CAP_PROP_POS_MSEC))
                metadata.write(json.dumps(row, allow_nan=False) + "\n")
                if args.save_masks:
                    _write_png(output / "masks" / f"{processed:06d}.png", result.road_mask)
                processed += 1
                valid += int(result.valid_image_path)
                durations.append(result.processing_ms)
                if args.show and _show(overlay):
                    stopped_by_user = True
                    break
                if args.max_frames and processed >= args.max_frames:
                    break
                ok, frame = capture.read()
            if not ok and args.camera is not None:
                raise OSError("webcam stopped delivering frames")
            if not ok and np.isfinite(expected_frames) and expected_frames > processed + 1:
                raise OSError(f"video ended early: decoded {processed} of approximately {expected_frames:.0f} frames")
        return {
            "frames": processed,
            "valid_image_paths": valid,
            "mean_processing_ms": float(np.mean(durations)),
            "source_fps_or_fallback": fps,
            "stopped_by_user": stopped_by_user,
        }
    finally:
        capture.release()
        if writer is not None:
            writer.release()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path, help="single image path")
    source.add_argument("--video", type=Path, help="video file path")
    source.add_argument("--camera", type=int, help="webcam device index, for example 0")
    parser.add_argument("--output", type=Path, help="a NEW output directory; existing directories are not overwritten")
    parser.add_argument("--config", type=Path, help="JSON DetectorConfig overrides")
    parser.add_argument("--max-width", type=int)
    parser.add_argument("--clearance-px", type=int, help="image pixel clearance, NOT robot width in meters")
    parser.add_argument("--goal-x", type=float, help="preferred horizontal direction, normalized 0..1")
    parser.add_argument("--no-temporal", action="store_true", help="process each frame independently")
    parser.add_argument("--max-frames", type=int, help="stop after this many video/webcam frames")
    parser.add_argument("--save-masks", action="store_true", help="save lossless PNG masks for each video frame")
    parser.add_argument("--probability", type=Path, help="image only: external float [0,1] HxW .npy map")
    parser.add_argument("--blocked-mask", type=Path, help="image only: grayscale obstacle mask, nonzero=blocked")
    parser.add_argument("--show", action="store_true", help="optional GUI; requires opencv-python instead of headless")
    args = parser.parse_args(argv)
    if args.max_frames is not None and args.max_frames <= 0:
        parser.error("--max-frames must be positive")
    if (args.probability or args.blocked_mask) and not args.image:
        parser.error("--probability and --blocked-mask are supported with --image only; use the Python API for video models")
    if args.camera is not None and not args.show and args.max_frames is None:
        parser.error("headless webcam capture requires --max-frames")
    if args.show:
        gui_line = next((line for line in cv2.getBuildInformation().splitlines() if line.strip().startswith("GUI:")), "")
        if "NONE" in gui_line or (sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))):
            parser.error("GUI is unavailable; omit --show or install a GUI-enabled OpenCV in a desktop environment")

    started = time.perf_counter()
    try:
        overrides = json.loads(args.config.read_text(encoding="utf-8")) if args.config else {}
        if not isinstance(overrides, dict):
            raise ValueError("config must contain a JSON object")
        for field, value in (("max_width", args.max_width), ("clearance_px", args.clearance_px), ("goal_x_fraction", args.goal_x)):
            if value is not None:
                overrides[field] = value
        if args.no_temporal:
            overrides["use_temporal"] = False
        try:
            config = DetectorConfig(**overrides)
        except TypeError as exc:
            raise ValueError(f"unknown configuration field: {exc}") from exc
        detector = RoadDetector(config)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        output = args.output or Path("outputs") / stamp
        summary = _run_image(args, detector, output) if args.image else _run_video(args, detector, output)
        summary["elapsed_seconds"] = time.perf_counter() - started
        summary["output_directory"] = str(output)
        _write_json(output / "summary.json", summary)
        print(json.dumps(summary, indent=2, allow_nan=False))
        return 0
    except (ValueError, OSError, cv2.error) as exc:
        print(f"Road detection failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if args.show:
            cv2.destroyAllWindows()

