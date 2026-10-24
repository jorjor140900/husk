"""Measure real ROS overlay delivery with a synthetic camera, without a GUI.

Requires a sourced ROS 2 environment plus the segmentation dependencies. Uses a
private topic namespace so it does not open or publish to the real camera. The
model is exercised for timing only; this does not evaluate perception accuracy.
"""

import argparse
import json
import math
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import time
import uuid


def probe(args):
    import cv2
    import numpy as np
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage, Image
    from std_msgs.msg import Header, String
    from husky_drivable_area.nodes import image_qos
    from husky_drivable_area.transport import fill_image

    namespace = "/husky_stream_probe_" + uuid.uuid4().hex[:10]
    rclpy.init(args=[])
    node = Node("probe", namespace=namespace)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    publisher = node.create_publisher(Image, "camera/image_raw", image_qos())
    received, ages, expected, sizes = {}, [], set(), []
    latest_reason = None
    cv2.setNumThreads(1)
    texture = np.random.default_rng(7).normal(0, 15, (480, 640, 3))
    frame = np.clip(120 + texture, 0, 255).astype(np.uint8)

    def stamp_key(stamp):
        return stamp.sec, stamp.nanosec

    def publish():
        header = Header(stamp=node.get_clock().now().to_msg(), frame_id="probe_camera_optical_frame")
        publisher.publish(fill_image(Image(), frame, header))

    def overlay(message):
        received[stamp_key(message.header.stamp)] = time.perf_counter()
        source = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        ages.append(node.get_clock().now().nanoseconds / 1e9 - source)
        sizes.append(len(message.data))
        if args.transport == "compressed":
            decoded = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
            if decoded is None or decoded.shape != frame.shape:
                raise ValueError("compressed overlay could not be decoded at the original dimensions")

    def status(message):
        nonlocal latest_reason
        row = json.loads(message.data)
        latest_reason = row.get("reason")
        subscriber_key = "overlay_compressed_subscribers" if args.transport == "compressed" else "overlay_raw_subscribers"
        if row.get(subscriber_key, 0) > 0:
            stamp = row["source_header"]["stamp"]
            expected.add((stamp["sec"], stamp["nanosec"]))

    image_type = CompressedImage if args.transport == "compressed" else Image
    topic = "drivable_area/overlay" + ("/compressed" if args.transport == "compressed" else "")
    image_sub = node.create_subscription(image_type, topic, overlay, image_qos())
    status_sub = node.create_subscription(String, "drivable_area/status", status, 10)
    timer = node.create_timer(1 / args.camera_fps, publish)
    # SIGUSR1 stops spin without invalidating the context during an in-flight
    # inference callback, allowing the probe's child process to exit cleanly.
    child_code = (
        "import signal\n"
        "def stop(signum, frame): raise KeyboardInterrupt()\n"
        "signal.signal(signal.SIGUSR1, stop)\n"
        "from husky_drivable_area.nodes import detector_main\n"
        "detector_main()\n"
    )
    command = [sys.executable, "-c", child_code, "--ros-args", "-r", "__ns:=" + namespace]
    for key, value in {
        "model_path": str(args.model.expanduser().resolve()), "device": "cpu",
        "image_topic": namespace + "/camera/image_raw", "cpu_threads": args.cpu_threads,
        "model_width": args.model_width, "model_height": args.model_height,
    }.items():
        command.extend(["-p", key + ":=" + json.dumps(value)])
    process = None
    try:
        process = subprocess.Popen(command, cwd=Path(__file__).resolve().parents[1])
        started = reported = time.perf_counter()
        while time.perf_counter() - started < args.seconds and process.poll() is None:
            executor.spin_once(timeout_sec=0.1)
            now = time.perf_counter()
            if now - reported > 5:
                print(json.dumps({"elapsed_sec": round(now - started, 1), "received_images": len(received),
                                  "processed_with_viewer": len(expected), "last_reason": latest_reason}), flush=True)
                reported = now
        timer.cancel()
        # Drain the last in-flight model output before counting missing frames.
        deadline = time.perf_counter() + 2
        while time.perf_counter() < deadline and process.poll() is None:
            executor.spin_once(timeout_sec=0.1)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGUSR1)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    timestamps = sorted(received.values())
    gaps = [b - a for a, b in zip(timestamps, timestamps[1:])]
    missing = expected - received.keys()
    summary = {
        "transport": args.transport, "source": "synthetic_textured_images_640x480",
        "processed_with_viewer": len(expected), "received_images": len(received),
        "missing_processed_images": len(missing),
        "median_payload_bytes": statistics.median(sizes) if sizes else None,
        "median_age_ms": statistics.median(ages) * 1000 if ages else None,
        "max_gap_sec": max(gaps) if gaps else None,
        "median_gap_sec": statistics.median(gaps) if gaps else None,
        "model_accuracy_evaluated": False, "viewer_gui_tested": False,
    }
    print(json.dumps(summary, indent=2), flush=True)
    return 1 if not received or (args.require_all_frames and missing) else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True, help="extracted local SegFormer directory")
    parser.add_argument("--transport", choices=("raw", "compressed"), default="compressed")
    parser.add_argument("--seconds", type=float, default=25)
    parser.add_argument("--cpu-threads", type=int, default=2)
    parser.add_argument("--camera-fps", type=float, default=15)
    parser.add_argument("--model-width", type=int, default=448)
    parser.add_argument("--model-height", type=int, default=256)
    parser.add_argument("--require-all-frames", action="store_true")
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds < 5 or not math.isfinite(args.camera_fps) or args.camera_fps <= 0:
        parser.error("seconds must be >= 5 and camera-fps must be finite and positive")
    if args.cpu_threads < 1 or any(size < 32 or size % 32 for size in (args.model_width, args.model_height)):
        parser.error("cpu-threads must be positive; model dimensions must be multiples of 32")
    return probe(args)


if __name__ == "__main__":
    raise SystemExit(main())
