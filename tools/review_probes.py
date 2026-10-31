"""Export draft sparse annotations and diagnose color-threshold tradeoffs.

Probes are interior points chosen by visual inspection. Their results are a
small, biased diagnostic sample, not a held-out accuracy estimate. Sparse label
masks contain 0=blocked, 1=allowed, 255=unlabeled and require human review.
"""

import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path

import cv2
import numpy as np

from road_navigation import DetectorConfig, RoadDetector


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--annotations", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    cv2.setNumThreads(1)
    manifest = {row["name"]: row for row in json.loads(args.manifest.read_text())}
    annotations = json.loads(args.annotations.read_text())
    ref_w, ref_h = annotations["reference_size_wh"]
    radius = annotations["patch_radius_reference_pixels"]
    args.output.mkdir(parents=True, exist_ok=False)
    frames = []
    dataset = []
    sheets = []
    for row in annotations["frames"]:
        source = manifest[row["clip"]]
        cap = cv2.VideoCapture(source["path"])
        try:
            cap.set(cv2.CAP_PROP_POS_FRAMES, row["frame"])
            ok, frame = cap.read()
            if not ok:
                raise OSError(f"cannot read {row['clip']} frame {row['frame']}")
        finally:
            cap.release()
        h, w = frame.shape[:2]
        mask = np.full((h, w), 255, np.uint8)
        preview = cv2.resize(frame, (ref_w, ref_h), interpolation=cv2.INTER_AREA)
        points = [(xy, 1, "allowed_floor") for xy in row["allowed"]]
        points += [(point["xy"], 0, point["kind"]) for point in row["blocked"]]
        for xy, label, kind in points:
            x, y = xy
            if not radius <= x < ref_w - radius or not radius <= y < ref_h - radius:
                raise ValueError(f"probe outside image: {xy}")
            x0, x1 = round((x-radius)*w/ref_w), round((x+radius+1)*w/ref_w)
            y0, y1 = round((y-radius)*h/ref_h), round((y+radius+1)*h/ref_h)
            mask[y0:y1, x0:x1] = label
            color = (0, 220, 0) if label else (0, 0, 255)
            cv2.circle(preview, (x, y), 7, color, 2)
        cv2.rectangle(preview, (0, 0), (ref_w, 29), (0, 0, 0), -1)
        text = f"{row['clip']} f{row['frame']} | draft probes: green=allowed red=blocked"
        cv2.putText(preview, text, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, .45, (255,255,255), 1)
        name = f"{row['clip']}-f{row['frame']:06d}"
        folder = args.output / name
        folder.mkdir()
        for path, image in ((folder/'image.png', frame), (folder/'sparse-label.png', mask), (folder/'review.png', preview)):
            if not cv2.imwrite(str(path), image):
                raise OSError(f"cannot write {path}")
        dataset.append({
            "image": f"{name}/image.png", "sparse_mask": f"{name}/sparse-label.png",
            "source_clip": row["clip"], "source_frame_index": row["frame"],
            "source_time_seconds": row["frame"] / source["fps"],
            "recording_group": "front-building-session-01", "split": "unassigned",
            "label_status": "draft_sparse_human_review_required",
        })
        frames.append((row, frame, points))
        sheets.append(cv2.resize(preview, (480, 270)))
    if sheets:
        if len(sheets) % 2:
            sheets.append(np.zeros_like(sheets[0]))
        contact = np.vstack([np.hstack(sheets[i:i+2]) for i in range(0,len(sheets),2)])
        if not cv2.imwrite(str(args.output / "probe-contact.jpg"), contact):
            raise OSError("cannot write probe contact sheet")
    sweep = []
    for tolerance in (6.0, 10.0, 14.0, 18.0, 22.0):
        config = DetectorConfig(chroma_tolerance=tolerance, use_temporal=False)
        counts = Counter()
        point_results = []
        for row, frame, points in frames:
            result = RoadDetector(config).process(frame)
            h, w = result.road_mask.shape
            for xy, label, kind in points:
                x, y = round(xy[0]*w/ref_w), round(xy[1]*h/ref_h)
                predicted_allowed = bool(result.road_mask[y, x])
                key = "allowed_retained" if label and predicted_allowed else "allowed_rejected" if label else "blocked_accepted" if predicted_allowed else "blocked_rejected"
                counts[key] += 1
                point_results.append({
                    "clip":row["clip"],"frame":row["frame"],"xy_reference":xy,
                    "label":label,"kind":kind,"predicted_allowed":predicted_allowed,
                    "appearance_score":float(result.probability[y,x])
                })
        sweep.append({"config":asdict(config),"probe_counts":dict(counts),"points":point_results})
    output = {
        "warning":"Draft sparse visual probes on the SAME session used for diagnosis; not dense ground truth or held-out accuracy.",
        "ignore_index":255, "classes":{"0":"blocked","1":"allowed"}, "human_review_required":True,
        "dataset":dataset,"diagnostic_color_sweep":sweep,
    }
    (args.output / "manifest-and-diagnostics.json").write_text(json.dumps(output,indent=2)+"\n",encoding="utf-8")
    for row in sweep:
        print(json.dumps({"chroma_tolerance":row["config"]["chroma_tolerance"],**row["probe_counts"]}))


if __name__ == "__main__":
    main()
