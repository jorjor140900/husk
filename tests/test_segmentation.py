"""Optional integration checks; run with requirements-segmentation installed."""

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import cv2
import numpy as np

AVAILABLE = all(importlib.util.find_spec(name) is not None for name in ("torch", "transformers", "PIL"))
ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(AVAILABLE, "optional segmentation dependencies are not installed")
class SegmentationTests(unittest.TestCase):
    def make_fixture(self, root, clip_count=1):
        import torch
        from transformers import SegformerConfig, SegformerForSemanticSegmentation, SegformerImageProcessor

        torch.set_num_threads(2)
        config = SegformerConfig(
            depths=[1, 1, 1, 1], hidden_sizes=[8, 16, 32, 64],
            num_attention_heads=[1, 1, 2, 4], sr_ratios=[8, 4, 2, 1],
            decoder_hidden_size=16, num_labels=3,
        )
        initial = root / "initial-fixture"
        SegformerForSemanticSegmentation(config).save_pretrained(initial)
        # ADE checkpoints normally reduce labels; this must be overridden.
        SegformerImageProcessor(do_reduce_labels=True, size={"height": 64, "width": 64}).save_pretrained(initial)
        data = root / "dataset"
        (data / "images").mkdir(parents=True)
        (data / "masks").mkdir()
        frame = np.full((64, 96, 3), (80, 120, 160), np.uint8)
        mask = np.zeros((64, 96), np.uint8)
        mask[:, 48:88] = 1
        mask[:, 88:] = 255
        self.assertTrue(cv2.imwrite(str(data / "images/a.png"), frame))
        self.assertTrue(cv2.imwrite(str(data / "masks/a.png"), mask))
        rows = [{"image": "images/a.png", "mask": "masks/a.png", "recording_group": "test-session", "source_clip": "part1.mov"}]
        if clip_count == 2:
            (data / "clip2/images").mkdir(parents=True)
            (data / "clip2/masks").mkdir()
            self.assertTrue(cv2.imwrite(str(data / "clip2/images/a.png"), frame))
            self.assertTrue(cv2.imwrite(str(data / "clip2/masks/a.png"), mask))
            rows.append({"image": "clip2/images/a.png", "mask": "clip2/masks/a.png", "recording_group": "test-session", "source_clip": "part2.mov"})
        (data / "manifest.json").write_text(json.dumps(rows))
        return initial, data

    def test_training_export_reload_and_cli_use_the_model_without_edge_barriers(self):
        from argparse import Namespace
        import torch
        from road_navigation.segmentation import SegmentationPredictor
        from tools.train_segmentation import PairedDataset, train
        from transformers import SegformerImageProcessor

        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            initial, dataset = self.make_fixture(root, clip_count=2)
            processor = SegformerImageProcessor.from_pretrained(initial, do_reduce_labels=False)
            sample = PairedDataset(dataset, processor, augment=False)[0]
            self.assertEqual(set(sample["labels"].unique().tolist()), {0, 1, 255})
            output = root / "model"
            report = train(Namespace(
                dataset=dataset, output=output, checkpoint=str(initial), device="cpu",
                epochs=1, batch_size=2, learning_rate=6e-5, height=64, width=64,
                seed=42, max_steps=1,
            ))
            self.assertEqual(report["optimizer_steps"], 1)
            self.assertEqual(report["samples"], 2)
            self.assertEqual(report["samples_by_clip"], {"part1.mov": 1, "part2.mov": 1})
            self.assertEqual(len(list((output / "training_previews").glob("*.jpg"))), 2)
            self.assertFalse(report["independent_validation_performed"])
            self.assertTrue(np.isfinite(report["epochs"][0]["mean_training_loss"]))
            self.assertTrue((output / "model.safetensors").is_file())
            predictor = SegmentationPredictor(output, "cpu")
            self.assertFalse(predictor.processor.do_reduce_labels)
            # Make the exported model prefer ground everywhere. A sharp stripe
            # would veto the old color/Canny backend; --model must bypass it.
            with torch.no_grad():
                predictor.model.decode_head.classifier.weight.zero_()
                predictor.model.decode_head.classifier.bias.copy_(torch.tensor([-10., 10.]))
            predictor.model.save_pretrained(output)
            frame = np.full((120, 160, 3), 120, np.uint8)
            frame[80:85] = 0
            resized_predictor = SegmentationPredictor(output, "cpu", input_size=(96, 64))
            with mock.patch.object(resized_predictor.model, "forward", wraps=resized_predictor.model.forward) as forward:
                probability = resized_predictor.predict_probability(frame)
                self.assertEqual(tuple(forward.call_args.kwargs["pixel_values"].shape), (1, 3, 64, 96))
            self.assertEqual(probability.shape, (120, 160))
            self.assertTrue(np.isfinite(probability).all())
            self.assertTrue(np.all((probability >= 0) & (probability <= 1)))
            image = root / "input.png"
            self.assertTrue(cv2.imwrite(str(image), frame))
            result_dir = root / "inference"
            result = subprocess.run([
                sys.executable, "RoadDetection.py", "--image", str(image),
                "--model", str(output), "--device", "cpu", "--model-size", "96", "64", "--output", str(result_dir),
            ], cwd=ROOT, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            metadata = json.loads((result_dir / "result.json").read_text())
            self.assertEqual(metadata["backend"], "segformer")
            summary = json.loads((result_dir / "summary.json").read_text())
            self.assertEqual(summary["model_input_size_wh"], [96, 64])
            saved = cv2.imread(str(result_dir / "mask.png"), cv2.IMREAD_GRAYSCALE)
            self.assertEqual(saved[60, 80], 255)

    def test_invalid_model_input_size_is_rejected(self):
        from road_navigation.segmentation import SegmentationPredictor

        for size in ((0, 64), (65, 64), (64,), (64, 64, 64)):
            with self.subTest(size=size), self.assertRaisesRegex(ValueError, "multiples of 32"):
                SegmentationPredictor("unused", "cpu", input_size=size)

    def test_invalid_mask_values_are_rejected_before_training(self):
        from tools.train_segmentation import PairedDataset
        from transformers import SegformerImageProcessor

        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            initial, dataset = self.make_fixture(root)
            self.assertTrue(cv2.imwrite(str(dataset / "masks/a.png"), np.full((64, 96), 128, np.uint8)))
            processor = SegformerImageProcessor.from_pretrained(initial, do_reduce_labels=False)
            with self.assertRaisesRegex(ValueError, "class IDs"):
                PairedDataset(dataset, processor)


if __name__ == "__main__":
    unittest.main()
