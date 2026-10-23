"""Pair CVAT Segmentation mask exports with zero-based source video frames.

No detector is run and no labels are invented. CVAT background remains ignored,
not blocked. All samples keep one recording group; no random split is created.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import re
import tempfile
import zipfile

import cv2
import numpy as np


def _json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def _png(path, value):
    if not cv2.imwrite(str(path), value):
        raise OSError(f"cannot write {path}")


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_cvat_masks(annotations):
    """Read RGB/palette class masks by labelmap colors, not grayscale values."""
    classes = {"drivable": 1, "non-drivable": 0, "background": 255}
    with zipfile.ZipFile(annotations) as archive:
        names = archive.namelist()
        if len(names) != len(set(names)):
            raise ValueError("duplicate ZIP entries")
        maps = [n for n in names if PurePosixPath(n).name == "labelmap.txt"]
        if len(maps) != 1:
            raise ValueError("expected one labelmap.txt")
        palette = []
        for line in archive.read(maps[0]).decode("utf-8-sig").splitlines():
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            fields = line.split(":")
            label = fields[0].strip().lower().replace("_", "-")
            if label not in classes or len(fields) < 2:
                raise ValueError(f"unsupported labelmap entry: {line}")
            rgb = tuple(int(v) for v in fields[1].split(","))
            if len(rgb) != 3 or any(v < 0 or v > 255 for v in rgb):
                raise ValueError(f"invalid RGB color for {label}")
            if any(entry["rgb"] == list(rgb) or entry["label"] == label for entry in palette):
                raise ValueError("duplicate label or ambiguous labelmap color")
            palette.append({"label": label, "rgb": list(rgb), "training_id": classes[label]})
        if {entry["label"] for entry in palette} != set(classes):
            raise ValueError("labelmap must contain drivable, non-drivable and background")

        masks = {}
        files = sorted(n for n in names if PurePosixPath(n).parent.name == "SegmentationClass" and n.endswith(".png"))
        if not files:
            raise ValueError("no SegmentationClass PNG masks found")
        for name in files:
            match = re.fullmatch(r"frame_(\d+)", PurePosixPath(name).stem)
            if match is None:
                raise ValueError(f"cannot infer zero-based source frame from {name}; do not guess frame offsets")
            index = int(match.group(1))
            if index in masks:
                raise ValueError(f"duplicate source frame {index}")
            color = cv2.imdecode(np.frombuffer(archive.read(name), np.uint8), cv2.IMREAD_UNCHANGED)
            if color is None or color.ndim != 3 or color.shape[2] != 3 or color.dtype != np.uint8:
                raise ValueError(f"expected an 8-bit RGB/palette class PNG: {name}")
            mask = np.full(color.shape[:2], 255, np.uint8)
            recognized = np.zeros(mask.shape, bool)
            for entry in palette:
                selected = np.all(color == entry["rgb"][::-1], axis=2)
                mask[selected] = entry["training_id"]
                recognized |= selected
            if not recognized.all():
                raise ValueError(f"mask has colors missing from labelmap.txt: {name}")
            masks[index] = mask
        return masks, palette


def match_clip_pairs(paths):
    """Match partN.zip with partN.mov/mp4/avi, rejecting missing/ambiguous pairs."""
    archives, videos = {}, {}
    for value in paths:
        path = Path(value)
        suffix = path.suffix.lower()
        if suffix not in (".zip", ".mov", ".mp4", ".avi"):
            continue
        target = archives if suffix == ".zip" else videos
        key = path.stem.casefold()
        if key in target:
            raise ValueError(f"ambiguous files for {path.stem}: {target[key].name}, {path.name}")
        target[key] = path
    if not archives and not videos:
        raise ValueError("upload annotation ZIPs and original videos, for example part1.zip + part1.mov")
    missing_videos = sorted(set(archives) - set(videos))
    missing_archives = sorted(set(videos) - set(archives))
    if missing_videos or missing_archives:
        raise ValueError(f"filenames must match by stem; missing videos: {missing_videos}; missing annotation ZIPs: {missing_archives}")
    return [(archives[name], videos[name]) for name in sorted(archives)]


def prepare_collection(pairs, output, recording_group):
    """Combine clips into one manifest while keeping equal frame names separate."""
    pairs, output = list(pairs), Path(output)
    if not pairs:
        raise ValueError("at least one annotation/video pair is required")
    if output.exists():
        raise FileExistsError(f"output already exists: {output}")
    stems = [Path(video).stem.casefold() for _, video in pairs]
    if len(stems) != len(set(stems)):
        raise ValueError("each source clip must have a unique name")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".cvat-collection-", dir=output.parent) as folder:
        staging = Path(folder) / "dataset"
        staging.mkdir()
        manifest, clips, thumbnails = [], [], []
        totals = {"non_drivable": 0, "drivable": 0, "ignore": 0}
        for clip_index, (annotations, video) in enumerate(pairs, 1):
            prefix = Path("clips") / f"clip_{clip_index:02d}"
            child = staging / prefix
            summary = prepare_dataset(annotations, video, child, recording_group)
            rows = json.loads((child / "manifest.json").read_text())
            for row in rows:
                row["image"] = (prefix / row["image"]).as_posix()
                row["mask"] = (prefix / row["mask"]).as_posix()
            manifest.extend(rows)
            clips.append({"directory": prefix.as_posix(), **summary})
            for key in totals:
                totals[key] += summary["pixel_counts"][key]
            for index in sorted({0, len(rows) // 2, len(rows) - 1}):
                row = rows[index]
                preview = cv2.imread(str(child / "previews" / Path(row["image"]).name))
                if preview is None:
                    raise ValueError("missing prepared preview")
                thumbnail = np.zeros((298, 480, 3), np.uint8)
                thumbnail[28:] = cv2.resize(preview, (480, 270), interpolation=cv2.INTER_AREA)
                caption = f"{Path(video).name} | frame {row['source_frame_index']}"
                cv2.putText(thumbnail, caption, (5, 18), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
                thumbnails.append(thumbnail)
        while len(thumbnails) % 3:
            thumbnails.append(np.zeros_like(thumbnails[0]))
        _png(staging / "contact-sheet.png", np.vstack([np.hstack(thumbnails[i:i + 3]) for i in range(0, len(thumbnails), 3)]))
        result = {
            "samples": len(manifest), "clip_count": len(clips), "clips": clips,
            "recording_group": recording_group, "splits_created": False,
            "training_ids": {"non_drivable": 0, "drivable": 1, "ignore": 255},
            "pixel_counts": totals, "trained_model": False,
            "pairing_requires_visual_review": True,
        }
        _json(staging / "manifest.json", manifest)
        _json(staging / "summary.json", result)
        (staging / "README.md").write_text(
            "# Combined segmentation dataset\n\n"
            "manifest.json combines all clips for ONE training run and ONE final model.\n"
            "Image/mask paths include clip subdirectories so equal frame numbers never overwrite each other.\n"
            "Masks are uint8 IDs: 0=non-drivable, 1=drivable, 255=ignore.\n"
            "Per-clip summaries and full previews remain in clips/clip_NN/. Review their alignment before training.\n"
            "Clips from one recording session keep the same recording group; no random validation split is created.\n",
            encoding="utf-8",
        )
        if output.exists():
            raise FileExistsError(f"output already exists: {output}")
        staging.rename(output)
        return result


def prepare_dataset(annotations, video, output, recording_group):
    annotations, video, output = Path(annotations), Path(video), Path(output)
    if output.exists():
        raise FileExistsError(f"output already exists: {output}")
    if not recording_group.strip():
        raise ValueError("recording_group must identify the original recording session")
    masks, palette = read_cvat_masks(annotations)
    if not video.is_file():
        raise FileNotFoundError(video)
    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise ValueError(f"cannot open video: {video}")
    try:
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("video FPS is unavailable; timestamps cannot be inferred")
        output.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix=".cvat-pairing-", dir=output.parent) as folder:
            staging = Path(folder) / "dataset"
            for subfolder in ("images", "masks", "previews"):
                (staging / subfolder).mkdir(parents=True, exist_ok=True)
            manifest, thumbnails = [], []
            totals = np.zeros(3, dtype=np.int64)
            for index in range(max(masks) + 1):
                ok, image = capture.read()
                if not ok:
                    raise ValueError(f"source video ends before required frame {index}")
                if index not in masks:
                    continue
                mask = masks[index]
                if image.shape[:2] != mask.shape:
                    raise ValueError(f"frame {index}: source shape {image.shape[:2]} differs from mask {mask.shape}; resizing would hide a pairing error")
                name = f"frame_{index:06d}.png"
                _png(staging / "images" / name, image)
                _png(staging / "masks" / name, mask)
                counts = [int(np.count_nonzero(mask == label)) for label in (0, 1, 255)]
                totals += counts
                manifest.append({
                    "image": "images/" + name, "mask": "masks/" + name,
                    "source_clip": video.name, "source_frame_index": index,
                    "source_time_seconds": index / fps,
                    "recording_group": recording_group, "split": "unassigned",
                    "label_status": "user_annotations_pending_quality_review",
                    "width": image.shape[1], "height": image.shape[0],
                    "pixel_counts": dict(zip(("non_drivable", "drivable", "ignore"), counts)),
                })
                preview = image.copy()
                for label, color in ((0, (0, 0, 230)), (1, (0, 210, 0))):
                    selected = mask == label
                    preview[selected] = (.65 * image[selected] + .35 * np.array(color)).astype(np.uint8)
                preview[mask == 255] = (.55 * image[mask == 255]).astype(np.uint8)
                _png(staging / "previews" / name, preview)
                thumbnail = np.zeros((298, 480, 3), np.uint8)
                thumbnail[28:] = cv2.resize(preview, (480, 270), interpolation=cv2.INTER_AREA)
                cv2.putText(thumbnail, f"frame {index} | green=drivable red=blocked dark=ignore", (5, 18), cv2.FONT_HERSHEY_SIMPLEX, .36, (255, 255, 255), 1)
                thumbnails.append(thumbnail)
            while len(thumbnails) % 3:
                thumbnails.append(np.zeros_like(thumbnails[0]))
            _png(staging / "contact-sheet.png", np.vstack([np.hstack(thumbnails[i:i + 3]) for i in range(0, len(thumbnails), 3)]))
            summary = {
                "samples": len(manifest), "source_video": video.name,
                "source_fps": fps, "source_frame_indices_zero_based": sorted(masks),
                "recording_group": recording_group, "splits_created": False,
                "training_ids": {"non_drivable": 0, "drivable": 1, "ignore": 255},
                "pixel_counts": dict(zip(("non_drivable", "drivable", "ignore"), totals.tolist())),
                "cvat_labelmap": palette,
                "annotations_sha256": _sha256(annotations), "source_video_sha256": _sha256(video),
                "pairing_rule": "frame_NNNNNN is the zero-based original video frame; no offset or resize is applied",
                "pairing_requires_visual_review": True,
                "trained_model": False,
            }
            _json(staging / "manifest.json", manifest)
            _json(staging / "summary.json", summary)
            (staging / "README.md").write_text(
                "# Paired segmentation pilot dataset\n\n"
                "images/ contains original-resolution video frames; masks/ contains matching uint8 PNG labels.\n"
                "Use masks as integer class IDs: 0=non-drivable, 1=drivable, 255=ignore. Do not normalize masks by 255.\n"
                "Resize masks only with nearest-neighbor interpolation. Do not train on previews or contact-sheet.png.\n"
                "CVAT background is unlabeled and remains ignored; it is never automatically treated as blocked.\n"
                "SegmentationObject masks are not semantic class labels and are deliberately not imported.\n"
                "Review previews to confirm source-frame alignment and annotation quality before training.\n"
                "All samples retain one recording group. No train/validation/test split or trained model is included.\n"
                "Collect another recording for an independent evaluation; random nearby-frame splits overestimate generalization.\n",
                encoding="utf-8",
            )
            if output.exists():
                raise FileExistsError(f"output already exists: {output}")
            staging.rename(output)
            return summary
    finally:
        capture.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument("--annotations", type=Path, help="one CVAT Segmentation mask ZIP without images")
    inputs.add_argument("--inputs", type=Path, nargs="+", help="multiple matching ZIP/video pairs, e.g. part1.zip part1.mov part2.zip part2.mov")
    parser.add_argument("--video", type=Path, help="the exact original video for --annotations")
    parser.add_argument("--output", type=Path, required=True, help="a new dataset directory")
    parser.add_argument("--recording-group", required=True, help="same name for clips from the same recording session")
    args = parser.parse_args()
    if args.annotations is not None and args.video is None:
        parser.error("--annotations requires --video")
    if args.inputs is not None and args.video is not None:
        parser.error("include all videos in --inputs instead of --video")
    try:
        if args.inputs is not None:
            result = prepare_collection(match_clip_pairs(args.inputs), args.output, args.recording_group)
        else:
            result = prepare_dataset(args.annotations, args.video, args.output, args.recording_group)
    except (OSError, ValueError, zipfile.BadZipFile) as exc:
        parser.exit(1, f"Dataset preparation failed: {exc}\n")
    print(json.dumps({"output": str(args.output), **result}, indent=2))


if __name__ == "__main__":
    main()
