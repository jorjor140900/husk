"""Optional SegFormer inference. Import only when a model is requested."""

from pathlib import Path

import cv2
import numpy as np
import torch
from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor


def choose_device(name="auto"):
    if name not in ("auto", "cpu", "cuda"):
        raise ValueError("device must be auto, cpu or cuda")
    if name == "cuda" and not torch.cuda.is_available():
        raise ValueError("CUDA is unavailable; use --device cpu or auto")
    return torch.device("cuda" if name != "cpu" and torch.cuda.is_available() else "cpu")


class SegmentationPredictor:
    def __init__(self, model_directory, device="auto", input_size=None):
        if input_size is not None:
            if len(input_size) != 2 or any(type(v) is not int or v < 32 or v % 32 for v in input_size):
                raise ValueError("model input width and height must be positive multiples of 32")
        folder = Path(model_directory)
        if not folder.is_dir():
            raise ValueError("--model must point to an extracted model directory, not a ZIP")
        self.device = choose_device(device)
        self.processor = SegformerImageProcessor.from_pretrained(folder, local_files_only=True, do_reduce_labels=False)
        if input_size is not None:
            self.processor.size = {"width": input_size[0], "height": input_size[1]}
        self.input_size = (self.processor.size["width"], self.processor.size["height"])
        self.model = SegformerForSemanticSegmentation.from_pretrained(folder, local_files_only=True)
        labels = {int(k): str(v).lower().replace("-", "_") for k, v in self.model.config.id2label.items()}
        if self.model.config.num_labels != 2 or labels != {0: "non_drivable", 1: "drivable"}:
            raise ValueError("model must have class 0=non_drivable and class 1=drivable")
        self.model.to(self.device).eval()

    @torch.inference_mode()
    def predict_probability(self, frame_bgr):
        if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3 or frame_bgr.dtype != np.uint8:
            raise ValueError("expected an HxWx3 uint8 BGR frame")
        image = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=image, return_tensors="pt")
        inputs = {key: value.to(self.device) for key, value in inputs.items()}
        logits = self.model(**inputs).logits
        logits = torch.nn.functional.interpolate(logits, size=frame_bgr.shape[:2], mode="bilinear", align_corners=False)
        return logits.softmax(dim=1)[0, 1].float().cpu().numpy()
