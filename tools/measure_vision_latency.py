"""Read-only camera/AI timing probe. No robot, map, TF or motor output needed.

Run from the checkout in a sourced ROS environment; no colcon rebuild needed.
Header ages start at the publisher's timestamp, not camera exposure. The USB
publisher approximates host read completion, so camera/backend buffering and
GUI rendering need a separate optical/stopwatch comparison.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time


def distribution(values):
    if not values:
        return None
    ordered=sorted(values)
    return {name:round(ordered[max(0,math.ceil(q*len(ordered))-1)],2)
            for name,q in (("p50",.5),("p95",.95),("max",1.))}


def source_stamp(header):
    stamp=header["stamp"]
    sec,nsec=stamp["sec"],stamp["nanosec"]
    if type(sec) is not int or type(nsec) is not int or sec<0 or not 0<=nsec<10**9:
        raise ValueError("invalid_source_stamp")
    return sec*10**9+nsec


class StreamTiming:
    def __init__(self):
        self.latest=None
        self.reset()

    def reset(self):
        # Keep the last observation across warmup; reset measurement counters.
        self.seen=set();self.ages=[];self.gaps=[];self.source_gaps=[]
        self.messages=self.invalid=self.future=self.regressions=0
        self.samples=self.closed=0;self.last_unique=None

    def observe(self,ns,ros_ns,mono,valid=True):
        self.messages+=1
        if not valid or type(ns) is not int or ns<=0:
            self.invalid+=1;self.latest=None
            return False
        self.latest=(ns,mono)
        if ns in self.seen:
            return False
        age=(ros_ns-ns)/1e6
        self.future+=int(age < -50)
        self.ages.append(age);self.seen.add(ns)
        if self.last_unique is not None:
            old_ns,old_mono=self.last_unique
            self.gaps.append((mono-old_mono)*1000)
            if ns>old_ns:
                self.source_gaps.append((ns-old_ns)/1e6)
            else:
                self.regressions+=1
        self.last_unique=(ns,mono)
        return True

    def sample_gate(self,ros_ns,mono,budget=.5):
        self.samples+=1
        fresh=False
        if self.latest is not None:
            ns,received=self.latest
            fresh=(-.05<=(ros_ns-ns)/1e9<=budget and 0<=mono-received<=budget)
        self.closed+=int(not fresh)

    def report(self,duration):
        return dict(messages=self.messages,unique_captures=len(self.seen),
            delivered_fps=round(len(self.seen)/duration,2),
            source_age_at_receive_ms=distribution(self.ages),
            unique_capture_delivery_gap_ms=distribution(self.gaps),
            source_capture_gap_ms=distribution(self.source_gaps),
            invalid_messages=self.invalid,future_clock_samples=self.future,
            timestamp_regressions=self.regressions,
            sampled_age_gate_closed_percent=round(100*self.closed/self.samples,2) if self.samples else None)


class VisionTiming:
    def __init__(self,overlay=False,reference=False):
        self.streams={key:StreamTiming() for key in ("camera","ai_mask")}
        if reference:self.streams["controller_reference"]=StreamTiming()
        if overlay:self.streams["overlay_publication"]=StreamTiming()
        self.metrics={};self.reasons=Counter()

    def reset(self):
        for stream in self.streams.values():stream.reset()
        self.metrics.clear();self.reasons.clear()

    def packet(self,text,ros_ns,mono):
        try:
            data=json.loads(text)
            if data.get("protocol")!=1 or data.get("valid") is not True:
                raise ValueError("invalid_segmentation")
            ns=source_stamp(data["source_header"])
            self.streams["ai_mask"].observe(ns,ros_ns,mono)
        except (ValueError,KeyError,TypeError,AttributeError):
            self.streams["ai_mask"].observe(0,ros_ns,mono,False)

    def status(self,text):
        try:
            data=json.loads(text)
            self.reasons[str(data.get("reason","unknown"))]+=1
            for key in ("input_age_ms","inference_ms","detector_ms","image_output_ms","frame_age_ms"):
                value=data.get(key)
                if type(value) in (int,float) and math.isfinite(value) and value>=0:
                    self.metrics.setdefault(key,[]).append(value)
        except (ValueError,TypeError,AttributeError):
            self.reasons["unreadable_status"]+=1

    def sample(self,ros_ns,mono):
        for stream in self.streams.values():stream.sample_gate(ros_ns,mono)

    def report(self,duration):
        streams={key:value.report(duration) for key,value in self.streams.items()}
        return dict(duration_sec=round(duration,2),image_age_budget_ms=500,gate_sample_period_ms=50,
            streams=streams,ai_reported_timings_ms={key:distribution(values) for key,values in self.metrics.items()},
            ai_status_reasons=dict(self.reasons),read_only=True,ready_for_control=False,
            limits=["Age is measured from source headers; USB headers approximate host read completion, not exposure.",
                    "No measurement of camera/backend pre-stamp buffering or GUI rendering/monitor latency.",
                    "Age-gate percentages are sampled at 20 Hz; they are not actual motor-stop measurements.",
                    "AI packet timing precedes ground projection/control. Controller reference is optional when no robot/goal/TF.",
                    "No image decoding/inference. Subscribing to segmentation requests its normal PNG publication."])


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration",type=float,default=20.)
    parser.add_argument("--image-topic",default="/camera/image_raw")
    parser.add_argument("--with-overlay",action="store_true",help="also request/observe compressed preview; can activate rendering work")
    parser.add_argument("--with-reference",action="store_true",help="also observe the controller reference when navigation/goal/TF are running")
    parser.add_argument("--overlay-topic",default="/husky_nav/vision_overlay/compressed")
    parser.add_argument("--output-dir",type=Path,default=Path("outputs/vision-latency"))
    args=parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 5<=args.duration<=120:
        parser.error("duration must be 5..120 seconds")
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from sensor_msgs.msg import Image,CompressedImage
        from nav_msgs.msg import Path as RosPath
        from std_msgs.msg import String
    except ImportError:
        parser.error("source ROS Jazzy and the workspace environment before running this script")
    rclpy.init();node=Node("vision_latency_measurement")
    timing=VisionTiming(args.with_overlay,args.with_reference)
    sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    def times():return node.get_clock().now().nanoseconds,time.monotonic()
    def image(message,key="camera"):
        stamp=message.header.stamp
        timing.streams[key].observe(stamp.sec*10**9+stamp.nanosec,*times())
    def reference(message):
        stamp=message.header.stamp
        timing.streams["controller_reference"].observe(stamp.sec*10**9+stamp.nanosec,*times(),valid=len(message.poses)>=2)
    subscriptions=[node.create_subscription(Image,args.image_topic,image,sensor),
        node.create_subscription(String,"/drivable_area/segmentation_packet",lambda m:timing.packet(m.data,*times()),1),
        node.create_subscription(String,"/drivable_area/status",lambda m:timing.status(m.data),1)]
    if args.with_reference:
        subscriptions.append(node.create_subscription(RosPath,"/husky_nav/vision_reference",reference,1))
    if args.with_overlay:
        subscriptions.append(node.create_subscription(CompressedImage,args.overlay_topic,
            lambda m:image(m,"overlay_publication"),sensor))
    collecting=False
    def sample():
        if collecting:timing.sample(*times())
    timer=node.create_timer(.05,sample)
    print("Warmup 2s, then measuring %.0fs. No robot/map required; no command publishers."%args.duration,flush=True)
    try:
        end=time.monotonic()+2.
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        timing.reset();collecting=True;started=time.monotonic();end=started+args.duration
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        report=timing.report(time.monotonic()-started)
        report["image_topic"]=args.image_topic
        report["overlay_requested"]=args.with_overlay
        if args.with_overlay:
            report["overlay_topic"]=args.overlay_topic
            report["limits"].append("Overlay subscription can activate preview/JPEG work; this measures publication, not GUI presentation.")
        args.output_dir.mkdir(parents=True,exist_ok=True)
        path=args.output_dir/("vision-latency-"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")+".json")
        path.write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
        print(json.dumps(report,indent=2,allow_nan=False));print("Report:",path)
        return 0 if all(len(timing.streams[key].seen)>=3 for key in ("camera","ai_mask")) else 2
    finally:
        node.destroy_node();rclpy.try_shutdown()


if __name__=="__main__":
    raise SystemExit(main())
