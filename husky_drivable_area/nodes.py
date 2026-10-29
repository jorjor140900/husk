"""ROS 2 Jazzy/Humble adapters for the existing detector and local SegFormer."""

import json
from dataclasses import replace
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point32, PolygonStamped
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header, String

from road_navigation import DetectorConfig, RoadDetector
from road_navigation.capture import LatestFrameCapture
from road_navigation.visualization import draw_result
from .transport import LatestInput, fill_compressed_image, fill_image, frame_age_seconds, image_to_bgr, status_payload
from .ground_geometry import CameraCalibration, CameraMount
from .ground_overlay import robot_axis_pixels, reference_seed
from .vision_grid import encode_segmentation
from .nav2_camera_path import project_nav2_path, draw_nav2_plan


def image_qos():
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.BEST_EFFORT)


class DrivableAreaNode(Node):
    def __init__(self):
        super().__init__("drivable_area")
        defaults = {
            "image_topic": "/camera/image_raw", "model_path": "", "device": "cpu",
            "model_width": 448, "model_height": 256, "config_file": "",
            "max_frame_age_sec": 2.0, "cpu_threads": 2, "overlay_jpeg_quality": 80,
            "camera_calibration": "", "camera_mount": "",
            "show_nav2_path": False, "nav2_path_topic": "/husky_nav/goal_plan", "overlay_max_fps": 0.0,
            "async_reference_preview": False, "reference_preview_fps": 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self._axis_geometry = None
        self._axis_cache = None
        if bool(value("camera_calibration")) != bool(value("camera_mount")):
            raise ValueError("overlay requires both camera_calibration and camera_mount")
        if value("camera_calibration"):
            import inspect
            if "robot_axis" not in inspect.signature(draw_result).parameters:
                raise ValueError("calibrated overlay requires visualization.py update: export with --update-code --update-visualization")
            self._axis_geometry = (CameraCalibration.load(value("camera_calibration")),
                                   CameraMount.load(value("camera_mount")))
        self._max_age = float(value("max_frame_age_sec"))
        if not 0 < self._max_age < float("inf"):
            raise ValueError("max_frame_age_sec must be finite and positive")
        model_path = value("model_path")
        if not model_path:
            raise ValueError("model_path is required: pass the extracted husky-segformer directory")
        cpu_threads = value("cpu_threads")
        if type(cpu_threads) is not int or cpu_threads < 1:
            raise ValueError("cpu_threads must be a positive integer")
        self._jpeg_quality = value("overlay_jpeg_quality")
        if type(self._jpeg_quality) is not int or not 1 <= self._jpeg_quality <= 100:
            raise ValueError("overlay_jpeg_quality must be an integer from 1 to 100")
        overrides = json.loads(Path(value("config_file")).expanduser().read_text()) if value("config_file") else {}
        if not isinstance(overrides, dict):
            raise ValueError("config_file must contain a DetectorConfig JSON object")
        self._detector = RoadDetector(DetectorConfig(**overrides))
        self._show_nav2_path = value("show_nav2_path")
        self._overlay_max_fps = float(value("overlay_max_fps"))
        if not 0 <= self._overlay_max_fps <= 30:
            raise ValueError("overlay_max_fps must be 0..30 (0 = uncapped)")
        self._last_overlay_at = float("-inf")
        self._nav2_path = self._nav2_goal_status = None
        self._nav2_goal_start_ns = 0
        self._async_reference_preview = value("async_reference_preview")
        self._reference_worker = None
        if self._async_reference_preview:
            if self._show_nav2_path:
                raise ValueError("choose either async pixel-reference preview or Nav2 plan overlay")
            from .reference_preview import ReferencePreview
            preview_config = replace(self._detector.config,generate_reference_path=True)
            self._reference_start_info = dict(source="configured_image_fraction",valid=True)
            if self._axis_geometry is not None:
                try:
                    self._reference_start_info = reference_seed(*self._axis_geometry,preview_config)
                    preview_config = replace(preview_config,
                        seed_x_fraction=self._reference_start_info["seed_x_fraction"])
                except ValueError as exc:
                    # A display geometry failure must not stop segmentation or
                    # silently invent a centered robot axis.
                    self._reference_start_info = dict(source="calibrated_robot_axis",valid=False,reason=str(exc))
                    preview_config = replace(preview_config,generate_reference_path=False)
            self._reference_config = preview_config
            self._reference_worker = ReferencePreview(preview_config,float(value("reference_preview_fps")))
            # Full-resolution, current segmentation remains in the inference
            # process. Legacy temporal flow and pixel planning are display work.
            self._detector = RoadDetector(replace(self._detector.config,
                                                  generate_reference_path=False,use_temporal=False))
        if self._show_nav2_path:
            self._max_age = min(self._max_age,.5)
            if self._axis_geometry is None or getattr(self._detector.config,"generate_reference_path",True):
                raise ValueError("Nav2 camera path requires calibration/mount and generate_reference_path=false")
            from nav_msgs.msg import Path as RosPath
            from geometry_msgs.msg import PoseStamped
            from rclpy.qos import DurabilityPolicy
            from tf2_ros import Buffer, TransformListener
            self._route_tf = Buffer()
            self._route_listener = TransformListener(self._route_tf,self)
            self._route_topic = value("nav2_path_topic")
            path_qos = image_qos()
            path_qos.reliability = ReliabilityPolicy.RELIABLE
            if self._route_topic == "/husky_nav/goal_plan":
                path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self._route_sub = self.create_subscription(RosPath,self._route_topic,self._receive_nav2_path,path_qos)
            self._route_goal_sub = self.create_subscription(String,"/husky_nav/goal_status",self._receive_nav2_goal,1)
            goal_qos = image_qos()
            goal_qos.reliability = ReliabilityPolicy.RELIABLE
            goal_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self._route_goal_pose_sub = self.create_subscription(PoseStamped,"/husky_nav/goal",self._receive_nav2_goal_pose,goal_qos)
        from road_navigation.segmentation import SegmentationPredictor
        import torch
        # Leave CPU time for image transport, the ROS executor, and the viewer.
        # Large thread pools can be slower than a small pool for this input size.
        torch.set_num_threads(cpu_threads)
        cv2.setNumThreads(1)
        self.get_logger().info("Loading local SegFormer model...")
        self._predictor = SegmentationPredictor(
            Path(model_path).expanduser(), value("device"),
            input_size=(value("model_width"), value("model_height")),
        )
        self._latest = LatestInput()
        self._last_invalid_at = float("-inf")
        self._last_processed_header = None
        self._overlay_pub = self.create_publisher(Image, "drivable_area/overlay", image_qos())
        self._compressed_pub = self.create_publisher(CompressedImage, "drivable_area/overlay/compressed", image_qos())
        self._mask_pub = self.create_publisher(Image, "drivable_area/mask", image_qos())
        self._clearance_pub = self.create_publisher(Image, "drivable_area/navigable_mask", image_qos())
        self._reference_pub = self.create_publisher(PolygonStamped, "drivable_area/reference_pixels", 1)
        self._status_pub = self.create_publisher(String, "drivable_area/status", 1)
        self._segmentation_pub = self.create_publisher(String, "drivable_area/segmentation_packet", 1)
        self._vegetation_pub = self.create_publisher(Image, "drivable_area/vegetation_veto", image_qos())
        # Inference and incoming images use separate callback groups/threads.
        # DDS depth=1 AND the application slot=1 prevent an inference backlog.
        self._input_group = MutuallyExclusiveCallbackGroup()
        self._processing_group = MutuallyExclusiveCallbackGroup()
        self._subscription = self.create_subscription(
            Image, value("image_topic"), self._latest.put, image_qos(),
            callback_group=self._input_group,
        )
        self._timer = self.create_timer(0.02, self._process_latest, callback_group=self._processing_group)
        if self._show_nav2_path:
            self._route_retry_timer = self.create_timer(.02, self._retry_nav2_display,
                                                       callback_group=self._processing_group)
        if self._async_reference_preview:
            self._reference_preview_timer = self.create_timer(.02,self._publish_reference_preview,
                                                              callback_group=self._processing_group)
        self.get_logger().info(
            f"Ready: {value('image_topic')} -> /drivable_area/*; "
            f"{self._predictor.device}, model input {self._predictor.input_size}, "
            f"torch threads {torch.get_num_threads()}; "
            f"green vegetation veto={getattr(self._detector.config, 'reject_green_vegetation', False)}; "
            f"image reference={getattr(self._detector.config, 'generate_reference_path', True)}. Pixel references only."
            f" Async image preview={self._async_reference_preview}."
        )

    def _publish_reference(self, header, points, payload):
        reference = PolygonStamped()
        reference.header = header or Header()
        # Ordered open polyline, NOT a closed polygon or a metric ROS path.
        reference.polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in points]
        self._reference_pub.publish(reference)
        published = self.get_clock().now().to_msg()
        payload["published_stamp"] = {
            "sec": published.sec, "nanosec": published.nanosec,
        }
        self._status_pub.publish(String(data=json.dumps(payload, allow_nan=False)))

    def _receive_nav2_path(self, message):
        self._nav2_path = message

    def _publish_reference_preview(self):
        answer = self._reference_worker.take()
        if answer is None:
            return
        _,key,result,error = answer
        if error is not None:
            self.get_logger().warning("Reference display worker: "+error,throttle_duration_sec=2.)
            return
        if self._overlay_pub.get_subscription_count()+self._compressed_pub.get_subscription_count()==0:
            return
        header = Header()
        header.frame_id,header.stamp.sec,header.stamp.nanosec = key
        source_ns = header.stamp.sec*10**9+header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-source_ns)/1e9
        if source_ns<=0 or not -.05<=age<=self._max_age:
            return
        start = getattr(self,"_reference_start_info",dict(source="configured_image_fraction",valid=True)).copy()
        if not start["valid"]:
            result.reason = start["reason"]
        else:
            configuration = getattr(self,"_reference_config",self._detector.config)
            height,width = result.frame_bgr.shape[:2]
            sx,sy = round((width-1)*configuration.seed_x_fraction),round((height-1)*configuration.seed_y_fraction)
            start.update(seed_xy=[sx,sy],mask_drivable=bool(result.road_mask[sy,sx]),
                         clearance_passed=bool(result.navigable_mask[sy,sx]),clearance_px=configuration.clearance_px)
        axis = ({"robot_axis":self._overlay_axis(result,header)}
                if self._axis_geometry is not None else {})
        image = draw_result(result,telemetry={"processing_ms":result.processing_ms,
                                             "host_frame_age_ms":age*1000},**axis)
        if result.reason=="start_blocked_or_too_narrow" and start["valid"]:
            cv2.drawMarker(image,tuple(start["seed_xy"]),(0,0,255),cv2.MARKER_TILTED_CROSS,10,1,cv2.LINE_AA)
        self._publish_nav2_display(image,header)
        self._reference_preview_info = dict(reason=result.reason,source_stamp_ns=source_ns,
                                            source_age_ms=age*1000,display_only=True,start=start)

    def _receive_nav2_goal_pose(self, message):
        stamp = message.header.stamp
        self._nav2_goal_start_ns = stamp.sec*10**9+stamp.nanosec

    def _clear_nav2_display(self, header, reason):
        self._pending_route_display = None
        preview = getattr(self,"_camera_preview",None)
        if preview is None:
            return  # No camera image exists yet; never manufacture a black frame.
        frame, header = preview
        age = (self.get_clock().now().nanoseconds-
               (header.stamp.sec*10**9+header.stamp.nanosec))/1e9
        key = (reason,header.frame_id,header.stamp.sec,header.stamp.nanosec,int(age))
        if getattr(self,"_last_route_clear",None)==key:
            return
        # Remove the route immediately once invalid. Repeated invalid camera
        # frames obey the display cap, but never keep an old yellow route.
        max_fps = getattr(self,"_overlay_max_fps",0)
        if (getattr(self,"_last_route_clear",None) is not None and max_fps and
                time.perf_counter()-self._last_overlay_at<1/max_fps):
            return
        image = frame.copy()  # Raw camera background, without old mask or plan.
        draw_nav2_plan(image,[],"NO LIVE PLAN: "+reason)
        cv2.putText(image,"CAMERA CAPTURE AGE %.0fms | DISPLAY ONLY" % (age*1000),
                    (7,40),cv2.FONT_HERSHEY_SIMPLEX,.43,(0,180,255),1)
        self._nav2_display_info = dict(enabled=True,drawn=False,reason=reason)
        self._publish_nav2_display(image,header)
        self._last_route_clear = key

    def _publish_nav2_display(self, image, header):
        if self._overlay_pub.get_subscription_count()>0:
            self._overlay_pub.publish(fill_image(Image(),image,header))
        if self._compressed_pub.get_subscription_count()>0:
            self._compressed_pub.publish(fill_compressed_image(CompressedImage(),image,header,self._jpeg_quality))
        self._last_overlay_at = time.perf_counter()

    def _retry_nav2_display(self):
        pending = getattr(self,"_pending_route_display",None)
        if pending is None:
            return
        background, header, raw_shape = pending
        overlay = background.copy()
        info = self._draw_nav2_path(overlay,header,raw_shape)
        self._nav2_display_info = info
        if info["reason"]=="waiting_for_capture_tf":
            return
        self._pending_route_display = None
        self._publish_nav2_display(overlay,header)

    def _receive_nav2_goal(self, message):
        try:
            data = json.loads(message.data)
            self._nav2_goal_status = (data,time.monotonic()) if isinstance(data,dict) else None
        except (ValueError,TypeError):
            self._nav2_goal_status = None

    def _draw_nav2_path(self, overlay, header, raw_shape=None):
        from rclpy.time import Time
        from tf2_ros import TransformException
        from .control import quaternion_matrix
        pixels = []
        info = dict(enabled=True,drawn=False,source_topic=self._route_topic,
                    source_stamp_ns=header.stamp.sec*10**9+header.stamp.nanosec)
        try:
            calibration,mount = self._axis_geometry
            if header.frame_id!=mount.camera_frame:
                raise ValueError("camera_frame_mismatch")
            if raw_shape is not None and tuple(raw_shape)!=(calibration.height,calibration.width):
                raise ValueError("camera_size_mismatch")
            source_ns = header.stamp.sec*10**9+header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds-source_ns)/1e9
            if source_ns<=0 or not -.05<=age<=.5:
                raise ValueError("stale_image_path_hidden")
            goal = self._nav2_goal_status
            if goal is None or time.monotonic()-goal[1]>2:
                raise ValueError("waiting_for_goal_status")
            state = goal[0]
            if self._route_topic=="/husky_nav/goal_plan" and state.get("planning_only") is not True:
                raise ValueError("motion_requires_native_plan_topic")
            expected = "path_ready_preview_only" if self._route_topic=="/husky_nav/goal_plan" else "goal_submitted"
            if state.get("reason")!=expected:
                reason = str(state.get("reason","no_current_goal"))
                if reason=="goal_failed":
                    reason += " | "+str(state.get("error_name",state.get("error_code","unknown")))
                raise ValueError(reason)
            route = self._nav2_path
            if route is None or not 2<=len(route.poses)<=4096:
                raise ValueError("waiting_for_nav2_plan")
            stamp_ns = route.header.stamp.sec*10**9+route.header.stamp.nanosec
            plan_age = (self.get_clock().now().nanoseconds-stamp_ns)/1e9
            if stamp_ns<=0 or not -.05<=plan_age<=120:
                raise ValueError("plan_expired_select_goal_again")
            if stamp_ns<getattr(self,"_nav2_goal_start_ns",0):
                raise ValueError("waiting_for_new_goal_plan")
            if not route.header.frame_id or any(p.header.frame_id not in ("",route.header.frame_id) for p in route.poses):
                raise ValueError("plan_frame_mismatch")
            if any(not np.isfinite(p.pose.position.z) for p in route.poses):
                raise ValueError("plan_position_invalid")
            tf = self._route_tf.lookup_transform(self._axis_geometry[1].ground_frame,route.header.frame_id,
                Time(nanoseconds=source_ns,clock_type=self.get_clock().clock_type))
            q,t = tf.transform.rotation,tf.transform.translation
            matrix = np.eye(4)
            matrix[:3,:3] = quaternion_matrix(q.x,q.y,q.z,q.w)
            matrix[:3,3] = [t.x,t.y,t.z]
            points = [[p.pose.position.x,p.pose.position.y] for p in route.poses]
            pixels = project_nav2_path(points,matrix,*self._axis_geometry)
            calibration = self._axis_geometry[0]
            pixels = (pixels+.5)*[overlay.shape[1]/calibration.width,overlay.shape[0]/calibration.height]-.5
            if not -.05<=(self.get_clock().now().nanoseconds-source_ns)/1e9<=.5:
                raise ValueError("stale_after_projection")
            info.update(reason="preview" if state.get("planning_only") else "global_plan",plan_age_sec=plan_age)
        except TransformException as exc:
            pixels = []
            info.update(reason="waiting_for_capture_tf",tf_error=str(exc)[:160])
        except (ValueError,TypeError,AttributeError,cv2.error) as exc:
            pixels = []
            info["reason"] = str(exc)[:90]
        label = info["reason"]
        if "plan_age_sec" in info:
            label += " | plan age %.1fs" % info["plan_age_sec"]
        info["drawn"] = draw_nav2_plan(overlay,pixels,label)
        if not info["drawn"] and info["reason"] in ("preview","global_plan"):
            info["reason"] = "plan_outside_camera_view"
            draw_nav2_plan(overlay,[],info["reason"])
        return info

    def _overlay_axis(self, result, header):
        if getattr(self, "_axis_geometry", None) is None:
            return None
        key = (tuple(result.frame_bgr.shape[:2]), tuple(result.input_shape), header.frame_id)
        if self._axis_cache is not None and self._axis_cache[0] == key:
            return self._axis_cache[1]
        try:
            points = robot_axis_pixels(*self._axis_geometry, *key)
        except ValueError as exc:
            self.get_logger().warning(f"Robot axis hidden: {exc}", throttle_duration_sec=2.0)
            # Configured calibration must never fall back to a centered axis.
            return []
        self._axis_cache = (key, points)
        return points

    def _invalidate(self, reason, header=None, age=None, timings=None):
        self._detector.reset()
        if getattr(self,"_reference_worker",None) is not None:
            self._reference_worker.reset()
        _, _, skipped = self._latest.state()
        payload = status_payload(
            None, header, age_seconds=age, skipped=skipped, reason=reason,
        )
        payload.update(timings or {})
        if getattr(self,"_show_nav2_path",False):
            payload["nav2_camera_path"] = dict(enabled=True,drawn=False,reason=reason)
        self._publish_reference(header, [], payload)
        publisher = getattr(self, "_segmentation_pub", None)
        if publisher is not None and publisher.get_subscription_count() > 0:
            publisher.publish(String(data=json.dumps(dict(protocol=1, valid=False, reason=reason))))
        if getattr(self,"_show_nav2_path",False):
            self._clear_nav2_display(header,reason)
        detail = f", age {age * 1000:.0f}ms" if age is not None else ""
        if timings and "inference_ms" in timings:
            detail += f", inference {timings['inference_ms']:.0f}ms"
        self.get_logger().warning(
            f"No fresh reference: {reason}{detail}", throttle_duration_sec=2.0,
        )
        self._last_processed_header = None
        self._last_invalid_at = time.perf_counter()

    def _process_latest(self):
        pending = self._latest.take()
        if pending is None:
            received_at, header, _ = self._latest.state()
            now = time.perf_counter()
            if now - received_at > self._max_age and now - self._last_invalid_at >= 1.0:
                self._invalidate("camera_timeout", header, now - received_at)
            return
        header = pending.message.header
        timings = {}
        age = None
        try:
            age = frame_age_seconds(pending, self.get_clock().now().nanoseconds)
            timings["input_age_ms"] = age * 1000
            if age > self._max_age:
                self._invalidate("stale_input", header, age, timings)
                return
            previous = self._last_processed_header
            # New camera/frame or backward replay timestamps start new history.
            if previous is not None and (
                header.frame_id != previous.frame_id or
                (header.stamp.sec, header.stamp.nanosec) < (previous.stamp.sec, previous.stamp.nanosec)
            ):
                self._detector.reset()
                if getattr(self,"_reference_worker",None) is not None:
                    self._reference_worker.reset()
            frame = image_to_bgr(pending.message)
            if getattr(self,"_show_nav2_path",False):
                self._camera_preview = (frame,header)
            started = time.perf_counter()
            probability = self._predictor.predict_probability(frame)
            inference_finished = time.perf_counter()
            timings["inference_ms"] = (inference_finished - started) * 1000
            result = self._detector.process(frame, drivable_probability=probability)
            result.backend = "segformer"
            processed_at = time.perf_counter()
            result.processing_ms = (processed_at - started) * 1000
            timings["detector_ms"] = (processed_at - inference_finished) * 1000
            timings["processing_ms"] = result.processing_ms
            age = frame_age_seconds(pending, self.get_clock().now().nanoseconds)
            if age > self._max_age:
                self._invalidate("stale_result", header, age, timings)
                return
            _, _, skipped = self._latest.state()
            # Publish the planning input before optional display projection,
            # JPEG encoding or GUI transport can add latency to it.
            publisher = getattr(self, "_segmentation_pub", None)
            if publisher is not None and publisher.get_subscription_count() > 0:
                source_ns = header.stamp.sec*10**9+header.stamp.nanosec
                source_age = (self.get_clock().now().nanoseconds-source_ns)/1e9
                if getattr(self,"_async_reference_preview",False) and not (
                        source_ns>0 and -.05<=source_age<=.5 and age<=.5):
                    packet = json.dumps(dict(protocol=1,valid=False,reason="stale_segmentation_before_preview"))
                else:
                    packet = encode_segmentation(result, header, self._detector.config.horizon_fraction)
                publisher.publish(String(data=packet))
            raw_subscribers = self._overlay_pub.get_subscription_count()
            jpeg_subscribers = self._compressed_pub.get_subscription_count()
            if getattr(self,"_async_reference_preview",False) and (raw_subscribers or jpeg_subscribers):
                self._reference_worker.submit((header.frame_id,header.stamp.sec,header.stamp.nanosec),
                                               frame,probability,result.processing_ms)
            timings["overlay_raw_subscribers"] = raw_subscribers
            timings["overlay_compressed_subscribers"] = jpeg_subscribers
            route_info = getattr(self,"_nav2_display_info",
                dict(enabled=getattr(self,"_show_nav2_path",False),drawn=False,reason="viewer_not_rendered"))
            max_fps = getattr(self,"_overlay_max_fps",0)
            render_due = max_fps==0 or time.perf_counter()-getattr(self,"_last_overlay_at",float("-inf"))>=1/max_fps
            if ((raw_subscribers > 0 or jpeg_subscribers > 0) and render_due and
                    not getattr(self,"_async_reference_preview",False)):
                axis_options = ({"robot_axis": self._overlay_axis(result, header)}
                                if getattr(self, "_axis_geometry", None) is not None else {})
                overlay = draw_result(result, telemetry={
                    "processing_ms": result.processing_ms, "host_frame_age_ms": age * 1000,
                }, **axis_options)
                if route_info["enabled"]:
                    background = overlay.copy()
                    route_info = self._draw_nav2_path(overlay,header,result.input_shape)
                    self._nav2_display_info = route_info
                    self._pending_route_display = ((background,header,result.input_shape)
                        if route_info["reason"]=="waiting_for_capture_tf" else None)
                self._last_overlay_shape = overlay.shape
                self._last_route_clear = None
                if raw_subscribers > 0:
                    self._overlay_pub.publish(fill_image(Image(), overlay, header))
                if jpeg_subscribers > 0:
                    self._compressed_pub.publish(fill_compressed_image(
                        CompressedImage(), overlay, header, self._jpeg_quality,
                    ))
                self._last_overlay_at = time.perf_counter()
            for publisher, mask in ((self._mask_pub, result.road_mask),
                                    (self._clearance_pub, result.navigable_mask)):
                if publisher.get_subscription_count() > 0:
                    publisher.publish(fill_image(Image(), mask, header))
            vegetation_publisher = getattr(self, "_vegetation_pub", None)
            vegetation_mask = getattr(result, "vegetation_mask", None)
            if vegetation_publisher is not None and vegetation_mask is not None and vegetation_publisher.get_subscription_count() > 0:
                vegetation_publisher.publish(fill_image(Image(), vegetation_mask, header))
            timings["image_output_ms"] = (time.perf_counter() - processed_at) * 1000
            age = frame_age_seconds(pending, self.get_clock().now().nanoseconds)
            if age > self._max_age:
                self._invalidate("stale_output", header, age, timings)
                return
            payload = status_payload(result, header, age_seconds=age, skipped=skipped)
            payload.update(timings)
            payload["nav2_camera_path"] = route_info
            if getattr(self,"_async_reference_preview",False):
                payload["reference_preview"] = dict(enabled=True,display_only=True,
                    worker=self._reference_worker.stats(),last_display=getattr(self,"_reference_preview_info",None))
            self._publish_reference(header, result.path, payload)
            self._last_processed_header = header
            self.get_logger().info(
                f"Frame age {age * 1000:.0f}ms | inference {timings['inference_ms']:.0f}ms | "
                f"detector {timings['detector_ms']:.0f}ms | {result.reason} | "
                f"preview subscribers raw={raw_subscribers}, compressed={jpeg_subscribers}",
                throttle_duration_sec=5.0,
            )
        except Exception as exc:
            self._invalidate("processing_error", header, age, timings)
            self.get_logger().error(f"Frame rejected: {type(exc).__name__}: {exc}")

    def destroy_node(self):
        if getattr(self,"_reference_worker",None) is not None:
            self._reference_worker.close()
        return super().destroy_node()


class WebcamNode(Node):
    def __init__(self):
        super().__init__("webcam")
        for name, value in {
            "camera_index": 2, "width": 640, "height": 480, "fps": 15.0,
            "frame_id": "camera_optical_frame", "image_topic": "/camera/image_raw",
            "capture_status_topic": "/camera/capture_status",
        }.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        fps = float(value("fps"))
        if not 0 < fps <= 120 or value("width") < 16 or value("height") < 16 or value("camera_index") < 0:
            raise ValueError("camera index >= 0, dimensions >= 16, and 0 < fps <= 120 are required")
        cv2.setNumThreads(1)
        camera = cv2.VideoCapture(value("camera_index"), cv2.CAP_V4L2)
        if not camera.isOpened():
            camera.release()
            raise OSError(f"cannot open /dev/video{value('camera_index')}; close the standalone camera program")
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, value("width"))
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, value("height"))
        camera.set(cv2.CAP_PROP_FPS, fps)
        self._capture = LatestFrameCapture(camera)
        self._frame_id = value("frame_id")
        self._publisher = self.create_publisher(Image, value("image_topic"), image_qos())
        self._capture_status_pub = self.create_publisher(String, value("capture_status_topic"), 1)
        self._capture_report_at = time.perf_counter()
        self._capture_report_frames = self._capture.diagnostics()["captured_frames"]
        self._publish_last_at = None
        self._delivery_window = {}
        self._timer = self.create_timer(1.0 / fps, self._publish_frame)
        self._capture_status_timer = self.create_timer(1.0, self._publish_capture_status)
        self.get_logger().info(
            f"Camera {value('camera_index')}: "
            f"{int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
            f"{int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))}, requested {fps:g} fps"
        )

    def _publish_frame(self):
        started = time.perf_counter()
        ok, frame = self._capture.read()
        after_read = time.perf_counter()
        if not ok:
            raise OSError("webcam stopped delivering frames")
        # Approximate HOST read completion, not hardware exposure time.
        delay_ns = int(max(0.0, time.perf_counter() - self._capture.last_frame_timestamp) * 1e9)
        now = self.get_clock().now()
        stamp = now - Duration(nanoseconds=min(delay_ns, max(0, now.nanoseconds)))
        header = Header(stamp=stamp.to_msg(), frame_id=self._frame_id)
        message = fill_image(Image(), frame, header)
        after_copy = time.perf_counter()
        self._publisher.publish(message)
        finished = time.perf_counter()
        window = self._delivery_window
        window["published_frames"] = window.get("published_frames", 0) + 1
        for key, seconds in (
            ("slot_wait_max_ms", after_read - started),
            ("message_copy_max_ms", after_copy - after_read),
            ("dds_publish_max_ms", finished - after_copy),
            ("callback_max_ms", finished - started),
            ("publish_gap_max_ms", finished - self._publish_last_at if self._publish_last_at is not None else 0),
        ):
            window[key] = max(window.get(key, 0.0), seconds * 1000)
        self._publish_last_at = finished

    def _publish_capture_status(self):
        now = time.perf_counter()
        elapsed = max(1e-9, now - self._capture_report_at)
        stats = self._capture.diagnostics()
        payload = dict(
            window_sec=elapsed,
            capture_fps=(stats["captured_frames"] - self._capture_report_frames) / elapsed,
            published_fps=self._delivery_window.get("published_frames", 0) / elapsed,
            capture=stats, delivery=dict(self._delivery_window),
            executor="single_threaded", read_only=True,
            note="Host read completion, not exposure; capture maxima are session-wide, delivery maxima are per window.",
        )
        self._capture_status_pub.publish(String(data=json.dumps(payload, allow_nan=False)))
        self._capture_report_at = now
        self._capture_report_frames = stats["captured_frames"]
        self._delivery_window.clear()

    def destroy_node(self):
        self._capture.release()
        return super().destroy_node()


def _run(node_type, args=None, *, single_threaded=False):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor() if single_threaded else MultiThreadedExecutor(num_threads=2)
    try:
        node = node_type()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def detector_main(args=None):
    _run(DrivableAreaNode, args)


def camera_main(args=None):
    # Capture already runs in its own native-read worker. All ROS callbacks are
    # mutually exclusive, so a worker-pool handoff adds contention, not parallelism.
    _run(WebcamNode, args, single_threaded=True)
