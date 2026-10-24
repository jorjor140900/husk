"""Fine-tune a pretrained SegFormer on paired image/mask data.

This first-pilot runner trains on all supplied samples. It reports training
loss, never validation accuracy; collect an independent recording to evaluate.
"""

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import random
import time

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset
from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor

from road_navigation.segmentation import SegmentationPredictor, choose_device


BASE_MODEL = "nvidia/segformer-b0-finetuned-ade-512-512"
LABELS = {0: "non_drivable", 1: "drivable"}


class PairedDataset(Dataset):
    def __init__(self, folder, processor, augment=True):
        self.folder = Path(folder).resolve()
        self.processor, self.augment = processor, augment
        self.rows = json.loads((self.folder / "manifest.json").read_text())
        if not isinstance(self.rows, list) or not self.rows:
            raise ValueError("dataset manifest must contain samples")
        for row in self.rows:
            self.read_pair(row)  # Fail before training on invalid masks or paths.

    def read_pair(self, row):
        paths = [(self.folder / row[key]).resolve() for key in ("image", "mask")]
        if any(not path.is_relative_to(self.folder) for path in paths):
            raise ValueError("dataset paths must remain inside its directory")
        image = cv2.imread(str(paths[0]), cv2.IMREAD_COLOR)
        mask = cv2.imread(str(paths[1]), cv2.IMREAD_UNCHANGED)
        if image is None or mask is None or mask.ndim != 2 or image.shape[:2] != mask.shape:
            raise ValueError(f"image/mask mismatch: {row['image']}")
        if mask.dtype != np.uint8 or not set(np.unique(mask)).issubset({0, 1, 255}) or not np.any(mask != 255):
            raise ValueError("masks must contain class IDs 0/1/255 and some labeled pixels")
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB), mask

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, index):
        image, mask = self.read_pair(self.rows[index])
        if self.augment:
            if random.random() < .5:
                image, mask = image[:, ::-1].copy(), mask[:, ::-1].copy()
            image = np.clip(image.astype(np.float32) * random.uniform(.8, 1.2) + random.uniform(-12, 12), 0, 255).astype(np.uint8)
        values = self.processor(images=image, segmentation_maps=mask, return_tensors="pt", do_reduce_labels=False)
        if not torch.any(values["labels"] != 255):
            raise ValueError("resizing removed every labeled pixel")
        return {key: value.squeeze(0) for key, value in values.items()}


def train(args):
    if args.epochs < 1 or args.batch_size < 1 or args.max_steps is not None and args.max_steps < 1:
        raise ValueError("epochs, batch size and max steps must be positive")
    if args.height < 32 or args.width < 32 or args.height % 32 or args.width % 32:
        raise ValueError("training height/width must be positive multiples of 32")
    if not np.isfinite(args.learning_rate) or args.learning_rate <= 0:
        raise ValueError("learning rate must be positive")
    if args.output.exists():
        raise FileExistsError(f"output already exists: {args.output}; choose a new folder")
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    device = choose_device(args.device)
    processor = SegformerImageProcessor.from_pretrained(
        args.checkpoint, do_reduce_labels=False,
        size={"height": args.height, "width": args.width},
    )
    dataset = PairedDataset(args.dataset, processor)
    model = SegformerForSemanticSegmentation.from_pretrained(
        args.checkpoint, num_labels=2, id2label=LABELS,
        label2id={label: index for index, label in LABELS.items()},
        semantic_loss_ignore_index=255, ignore_mismatched_sizes=True,
    ).to(device)
    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=True, num_workers=0)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.learning_rate, weight_decay=.01)
    scaler = torch.amp.GradScaler("cuda", enabled=device.type == "cuda")
    args.output.mkdir(parents=True, exist_ok=False)
    report = {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "base_checkpoint": str(args.checkpoint), "samples": len(dataset),
        "recording_groups": sorted({row.get("recording_group", "unassigned") for row in dataset.rows}),
        "samples_by_clip": dict(Counter(row.get("source_clip", "unspecified") for row in dataset.rows)),
        "device": str(device), "seed": args.seed, "training_size_hw": [args.height, args.width],
        "class_ids": LABELS, "ignore_index": 255,
        "independent_validation_performed": False,
        "metrics_scope": "training loss only; no accuracy or generalization claim",
        "epochs": [], "optimizer_steps": 0,
    }
    started, steps = time.perf_counter(), 0
    print(f"Training {len(dataset)} samples on {device}. No independent validation set.", flush=True)
    for epoch in range(args.epochs):
        model.train()
        losses = []
        for batch in loader:
            batch = {key: value.to(device) for key, value in batch.items()}
            optimizer.zero_grad(set_to_none=True)
            with torch.autocast(device_type=device.type, enabled=device.type == "cuda"):
                loss = model(**batch).loss
            if not torch.isfinite(loss):
                raise RuntimeError("training loss is not finite; check label masks")
            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()
            losses.append(float(loss.detach().cpu()))
            steps += 1
            if len(losses) == 1 or steps % 5 == 0:
                print(f"Epoch {epoch + 1}: batch {len(losses)}/{len(loader)}, loss {losses[-1]:.4f}", flush=True)
            if args.max_steps is not None and steps >= args.max_steps:
                break
        row = {"epoch": epoch + 1, "mean_training_loss": float(np.mean(losses))}
        report["epochs"].append(row)
        report["optimizer_steps"] = steps
        report["elapsed_seconds"] = time.perf_counter() - started
        model.save_pretrained(args.output, safe_serialization=True)
        processor.save_pretrained(args.output)
        (args.output / "training_report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(f"Epoch {epoch + 1}/{args.epochs}: training loss {row['mean_training_loss']:.4f}; checkpoint saved", flush=True)
        if args.max_steps is not None and steps >= args.max_steps:
            break
    # Qualitative examples are training samples, explicitly not validation.
    del optimizer, scaler, model
    if device.type == "cuda":
        torch.cuda.empty_cache()
    predictor = SegmentationPredictor(args.output, str(device))
    previews = args.output / "training_previews"
    previews.mkdir()
    preview_rows, seen = [], set()
    for row in dataset.rows:
        clip = row.get("source_clip", "unspecified")
        if clip not in seen:
            preview_rows.append(row)
            seen.add(clip)
        if len(preview_rows) >= 6:
            break
    if len(preview_rows) == 1:
        preview_rows = dataset.rows[:3]
    for index, row in enumerate(preview_rows):
        image = cv2.imread(str(dataset.folder / row["image"]))
        probability = predictor.predict_probability(image)
        predicted = image.copy()
        selected = probability >= .5
        predicted[selected] = (.65 * image[selected] + .35 * np.array([0, 210, 0])).astype(np.uint8)
        comparison = np.hstack([image, predicted])
        cv2.putText(comparison, "TRAINING SAMPLE | left=input right=model; not validation", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 2)
        destination = previews / (f"{index:02d}-" + Path(row["image"]).stem + ".jpg")
        if not cv2.imwrite(str(destination), comparison):
            raise OSError(f"cannot write {destination}")
    print(f"Model saved to {args.output}. Download the complete folder, including preprocessor_config.json.", flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--checkpoint", default=BASE_MODEL)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--learning-rate", type=float, default=6e-5)
    parser.add_argument("--height", type=int, default=384)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--max-steps", type=int, help="optional short pipeline check, not a complete training run")
    args = parser.parse_args()
    try:
        train(args)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f"Training failed: {exc}\n")


if __name__ == "__main__":
    main()
