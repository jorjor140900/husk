"""Review whole clips, export baseline comparisons and unlabeled source frames.

Run from the repository root: python -m tools.review_videos --manifest ...
All frames are processed consecutively; sampling only controls the dataset
export, never the temporal detector. Descriptive metrics are not accuracy.
"""

import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess

import cv2
import numpy as np

from road_navigation import DetectorConfig, RoadDetector
from road_navigation.visualization import draw_result


def _json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _png(path, image):
    if not cv2.imwrite(str(path), image):
        raise OSError(f"cannot write {path}")


def review_clip(job):
    item, output, config_dict, sample_every, recording_group = job
    cv2.setNumThreads(1)
    output = Path(output)
    folder = output / item["name"]
    folder.mkdir()
    images = folder / "unlabeled_images"
    images.mkdir()
    cap = cv2.VideoCapture(item["path"])
    if not cap.isOpened():
        raise OSError(f"cannot open {item['path']}")
    detector = RoadDetector(DetectorConfig(**config_dict))
    sample_step = max(1, round(sample_every * item["fps"]))
    preview_indices = {min(item["frames"] - 1, round((item["frames"] - 1) * f)) for f in (0, .18, .38, .58, .78, .97)}
    previews, samples, durations, jumps, coverages = [], [], [], [], []
    reasons = Counter()
    valid = frame_index = 0
    previous_target = None
    previous_valid = None
    transitions = 0
    encoder = None
    try:
        with (folder / "encoder.log").open("w") as log, (folder / "frames.jsonl").open("w", encoding="utf-8") as rows:
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                result = detector.process(frame)
                overlay = draw_result(result)
                raw = result.frame_bgr.copy()
                cv2.rectangle(raw, (0, 0), (raw.shape[1], 30), (0, 0, 0), -1)
                text = f"{item['name']} | {frame_index / item['fps']:.2f}s | ORIGINAL"
                cv2.putText(raw, text, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, .52, (255, 255, 255), 1)
                comparison = np.hstack([raw, overlay])
                h, w = comparison.shape[:2]
                comparison = cv2.copyMakeBorder(comparison, 0, h % 2, 0, w % 2, cv2.BORDER_CONSTANT)
                if encoder is None:
                    encoder = subprocess.Popen([
                        "ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin",
                        "-f", "rawvideo", "-pixel_format", "bgr24", "-video_size", f"{comparison.shape[1]}x{comparison.shape[0]}",
                        "-framerate", str(item["fps"]), "-i", "pipe:0", "-an",
                        "-c:v", "libx264", "-preset", "fast", "-crf", "23", "-threads", "1",
                        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(folder / "comparison.mp4")
                    ], stdin=subprocess.PIPE, stderr=log)
                encoder.stdin.write(comparison.tobytes())

                gray = cv2.cvtColor(result.frame_bgr, cv2.COLOR_BGR2GRAY)
                row = result.metadata()
                row["frame_index"] = frame_index
                row["source_time_seconds"] = frame_index / item["fps"]
                row["blur_laplacian_variance"] = float(cv2.Laplacian(gray, cv2.CV_64F).var())
                row["mean_luma"] = float(gray.mean())
                roi = result.road_mask[int(gray.shape[0] * detector.config.horizon_fraction):]
                coverage = float((roi > 0).mean())
                row["roi_mask_coverage"] = coverage
                target = None
                if result.valid_image_path:
                    look_y = round(gray.shape[0] * .65)
                    where = result.path[result.path[:, 1] == look_y]
                    if len(where):
                        target = int(where[0, 0])
                row["target_x_at_65_percent_height"] = target
                if target is not None and previous_target is not None:
                    jumps.append(abs(target - previous_target))
                previous_target = target
                if previous_valid is not None and result.valid_image_path != previous_valid:
                    transitions += 1
                previous_valid = result.valid_image_path
                durations.append(result.processing_ms)
                coverages.append(coverage)
                reasons[result.reason] += 1
                valid += int(result.valid_image_path)
                rows.write(json.dumps(row, allow_nan=False) + "\n")
                if frame_index in preview_indices:
                    image = cv2.resize(overlay, (480, 270), interpolation=cv2.INTER_AREA)
                    cv2.putText(image, f"{item['name']} | {frame_index / item['fps']:.2f}s", (7, 261), cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 0, 255), 1)
                    previews.append(image)
                    _png(folder / f"overlay-f{frame_index:06d}.png", overlay)
                    _png(folder / f"mask-f{frame_index:06d}.png", result.road_mask)
                if frame_index % sample_step == 0:
                    image_path = images / f"{item['name']}-f{frame_index:06d}.png"
                    _png(image_path, frame)
                    samples.append({
                        "image": str(image_path.relative_to(output)),
                        "source_clip": item["name"], "source_frame_index": frame_index,
                        "source_time_seconds": frame_index / item["fps"],
                        "recording_group": recording_group, "split": "unassigned",
                        "label_status": "unlabeled", "width": frame.shape[1], "height": frame.shape[0],
                        "blur_laplacian_variance_at_processed_size": row["blur_laplacian_variance"],
                    })
                frame_index += 1
            if frame_index != item["frames"]:
                raise OSError(f"{item['name']}: decoded {frame_index}, expected {item['frames']} frames")
            if encoder is not None:
                encoder.stdin.close()
                if encoder.wait() != 0:
                    raise OSError(f"ffmpeg failed; see {folder / 'encoder.log'}")
    finally:
        cap.release()
        if encoder is not None and encoder.poll() is None:
            if not encoder.stdin.closed:
                encoder.stdin.close()
            encoder.wait()
    if len(previews) == 6:
        _png(folder / "contact.png", np.vstack([np.hstack(previews[:3]), np.hstack(previews[3:])]))
    summary = {
        "name": item["name"], "frames_processed": frame_index,
        "duration_seconds": frame_index / item["fps"], "valid_image_path_frames": valid,
        "reason_counts": dict(reasons), "path_availability_transitions": transitions,
        "mean_roi_mask_coverage": float(np.mean(coverages)),
        "mean_detector_ms": float(np.mean(durations)), "p95_detector_ms": float(np.percentile(durations, 95)),
        "mean_raw_target_movement_px": float(np.mean(jumps)) if jumps else None,
        "dataset_images": len(samples),
        "metric_warning": "Availability, heuristic quality and raw target movement are NOT accuracy or motion-compensated stability scores.",
    }
    _json(folder / "summary.json", summary)
    return summary, samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True, help="JSON list with name, path, frames, fps")
    parser.add_argument("--output", type=Path, required=True, help="new output directory")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--sample-every", type=float, default=2.0)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--recording-group", default="same-session-unverified")
    args = parser.parse_args()
    if not math.isfinite(args.sample_every) or args.sample_every <= 0 or args.workers < 1:
        parser.error("sample interval and workers must be positive")
    config = DetectorConfig(**json.loads(args.config.read_text())) if args.config else DetectorConfig()
    manifest = json.loads(args.manifest.read_text())
    names = set()
    for item in manifest:
        if not re.fullmatch(r"[A-Za-z0-9_-]+", item["name"]) or item["name"] in names:
            parser.error("clip names must be unique alphanumeric names (with - or _)")
        names.add(item["name"])
        if not Path(item["path"]).is_file() or item["frames"] < 1 or not math.isfinite(item["fps"]) or item["fps"] <= 0:
            parser.error("manifest requires existing clips, positive frame counts and finite positive FPS")
    if not manifest:
        parser.error("manifest contains no clips")
    args.output.mkdir(parents=True, exist_ok=False)
    _json(args.output / "config.json", asdict(config))
    # Keep the exact baseline code used, so future edits do not erase provenance.
    snapshot = args.output / "source_snapshot"
    snapshot.mkdir()
    hashes = {}
    for source in Path("road_navigation").glob("*.py"):
        shutil.copy2(source, snapshot / source.name)
        hashes[str(source)] = hashlib.sha256(source.read_bytes()).hexdigest()
    _json(args.output / "source_sha256.json", hashes)
    jobs = [(item, str(args.output), asdict(config), args.sample_every, args.recording_group) for item in manifest]
    summaries, samples = [], []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for summary, selected in pool.map(review_clip, jobs):
            summaries.append(summary)
            samples.extend(selected)
            print(json.dumps(summary), flush=True)
    _json(args.output / "summary.json", summaries)
    _json(args.output / "dataset_manifest.json", samples)
    (args.output / "DATASET_README.txt").write_text(
        "These are unannotated source frames, not ground truth. No mask generated by the baseline is a label.\n"
        "All four supplied clips appear to share one session/site; keep recording_group together when splitting data.\n"
        "Collect independent sessions/sites for a credible held-out test. Agree on carpet/sidewalk/curb policy before labeling.\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
