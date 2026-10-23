"""Summarize recorded controller errors; save a distinct report for each run."""

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import uuid


def summarize_tracking(errors):
    errors = list(errors)
    if not errors or any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in errors):
        raise ValueError("tracking errors must be nonempty finite nonnegative numbers")
    tail = errors[-max(1, math.ceil(len(errors) / 10)):]
    scale = max(errors)
    rmse = scale * math.sqrt(sum((v/scale)**2 for v in errors) / len(errors)) if scale else 0.0
    return {"samples": len(errors), "initial_m": errors[0], "final_m": errors[-1],
            "rmse_m": rmse, "max_m": scale,
            "tail_mean_m": math.fsum(v / len(tail) for v in tail)}


def write_report(directory, report):
    content = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    directory = Path(directory).expanduser().absolute()
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "COLCON_IGNORE").touch()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    path = directory / f"control-{stamp}-{uuid.uuid4().hex[:8]}.json"
    with path.open("x", encoding="utf-8") as output:
        output.write(content)
    return path
