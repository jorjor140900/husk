#!/usr/bin/env python
"""Read-only ROS1 inventory. Does not advertise robot commands/services."""
from __future__ import print_function

import argparse
from datetime import datetime
import json
import math
import os
import platform
import threading
import time
import uuid

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image, Joy, LaserScan
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from husky_link_core.protocol import wire


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default=".")
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args(rospy.myargv()[1:])
    if not 1 <= args.duration <= 30:
        raise ValueError("duration must be 1..30 seconds")
    rospy.init_node("husky_vision_inventory", anonymous=True)
    topics = rospy.get_published_topics("/")
    report = {"recorded_at_utc": datetime.utcnow().isoformat()+"Z", "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
              "python_version": platform.python_version(), "topics": topics, "odometry": {}, "scans": {}, "tf_edges": [],
              "tf_samples": {}, "velocity_interface_parameters": {}, "joy": {}, "bool_topics": {}, "cameras": {}}
    report["system_state"] = rospy.get_master().getSystemState()[2]
    report["registered_topic_types"] = rospy.get_master().getTopicTypes()[2]
    for name in rospy.get_param_names():
        if name.endswith(("/cmd_vel_timeout", "/twist_mux/topics", "/cmd_vel_mux/subscribers", "/autorepeat_rate")):
            report["velocity_interface_parameters"][name] = rospy.get_param(name)
    lock, edges = threading.Lock(), set()

    def odometry(message, topic):
        with lock:
            report["odometry"][topic] = {"frame_id": message.header.frame_id, "child_frame_id": message.child_frame_id,
                                         "stamp_secs": message.header.stamp.to_sec()}

    def scan(message, topic):
        with lock:
            report["scans"][topic] = {"frame_id": message.header.frame_id, "count": len(message.ranges),
                "angle_min": message.angle_min, "angle_max": message.angle_max, "angle_increment": message.angle_increment,
                "time_increment": message.time_increment, "scan_time": message.scan_time,
                "range_min": message.range_min, "range_max": message.range_max,
                "nan_count": sum(math.isnan(v) for v in message.ranges), "inf_count": sum(math.isinf(v) for v in message.ranges)}

    def tf(message, is_static):
        with lock:
            for t in message.transforms:
                edges.add((t.header.frame_id, t.child_frame_id))
                if len(report["tf_samples"]) < 1000:
                    p, q = t.transform.translation, t.transform.rotation
                    report["tf_samples"][t.child_frame_id] = {"parent": t.header.frame_id, "static": is_static,
                        "translation_m": [p.x, p.y, p.z], "rotation_xyzw": [q.x, q.y, q.z, q.w]}

    def joy(message, topic):
        with lock:
            count = report["joy"].get(topic, {}).get("messages_observed", 0)+1
            report["joy"][topic] = {"button_count": len(message.buttons), "buttons": list(message.buttons),
                                   "axes": list(message.axes), "stamp_secs": message.header.stamp.to_sec(), "messages_observed": count}

    def boolean(message, topic):
        with lock:
            report["bool_topics"][topic] = message.data

    def camera(message, info):
        topic, compressed = info
        data = {"frame_id": message.header.frame_id, "stamp_secs": message.header.stamp.to_sec()}
        if compressed:
            data.update(format=message.format, compressed_bytes=len(message.data))
        else:
            data.update(width=message.width, height=message.height, encoding=message.encoding)
        with lock:
            report["cameras"][topic] = data

    subs = [rospy.Subscriber("/tf", TFMessage, tf, callback_args=False, queue_size=10),
            rospy.Subscriber("/tf_static", TFMessage, tf, callback_args=True, queue_size=10)]
    camera_subscriptions = 0
    for name, kind in topics:
        if kind == "nav_msgs/Odometry" and len(subs) < 20:
            subs.append(rospy.Subscriber(name, Odometry, odometry, callback_args=name, queue_size=1))
        elif kind == "sensor_msgs/LaserScan" and len(subs) < 20:
            subs.append(rospy.Subscriber(name, LaserScan, scan, callback_args=name, queue_size=1))
        elif kind == "sensor_msgs/Joy" and len(subs) < 25:
            subs.append(rospy.Subscriber(name, Joy, joy, callback_args=name, queue_size=1))
        elif kind == "std_msgs/Bool" and len(subs) < 40:
            subs.append(rospy.Subscriber(name, Bool, boolean, callback_args=name, queue_size=1))
        elif kind in ("sensor_msgs/Image", "sensor_msgs/CompressedImage") and camera_subscriptions < 2:
            compressed = kind == "sensor_msgs/CompressedImage"
            subs.append(rospy.Subscriber(name, CompressedImage if compressed else Image, camera,
                                        callback_args=(name, compressed), queue_size=1))
            camera_subscriptions += 1
    end = time.time()+args.duration
    while not rospy.is_shutdown() and time.time() < end:
        time.sleep(0.05)
    with lock:
        report["tf_edges"] = sorted(edges)
        content = json.dumps(wire(report), indent=2, allow_nan=False).encode("utf-8")
    if not os.path.isdir(args.output_dir):
        os.makedirs(args.output_dir)
    filename = os.path.join(args.output_dir, "robot-inventory-"+datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")+"-"+uuid.uuid4().hex[:8]+".json")
    descriptor = os.open(filename, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, "wb") as target:
        target.write(content)
    print(filename)


if __name__ == "__main__":
    main()
