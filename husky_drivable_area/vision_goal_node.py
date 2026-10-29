"""Goal-directed image references and private pursuit requests; no driver output."""
from collections import OrderedDict
import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from road_navigation.detector import DetectorConfig
from .control import ControllerConfig, CommandLimiter, Demand, fresh_stamp, pursuit, quaternion_matrix, validate_path
from .controller_node import stamp_ns
from .ground_geometry import CameraCalibration, CameraMount, project_path
from .ground_overlay import reference_seed
from .transport import image_to_bgr, fill_compressed_image
from .vision_grid import decode_segmentation
from .vision_goal import VisionReferenceConfig, sample_ground_labels, route_direction, plan_vision_reference
from .vision_goal_overlay import draw_vision_reference


class VisionGoalNode(Node):
    def __init__(self):
        super().__init__("vision_goal_controller")
        defaults=dict(camera_calibration="",camera_mount="",base_frame="base_link",odom_frame="odom",
            image_topic="/camera/image_raw",half_width_m=.33,side_margin_m=.08)
        for key,value in defaults.items():
            self.declare_parameter(key,value)
        value=lambda key:self.get_parameter(key).value
        self.camera=CameraCalibration.load(value("camera_calibration"))
        self.mount=CameraMount.load(value("camera_mount"))
        self.base,self.odom_frame=value("base_frame"),value("odom_frame")
        self.config=VisionReferenceConfig(half_width_m=value("half_width_m"),side_margin_m=value("side_margin_m"))
        seed=reference_seed(self.camera,self.mount,DetectorConfig(max_width=max(640,self.camera.width)))
        u,v=seed["seed_xy"]
        points,reason=project_path([[u,v],[u,v-1]],[self.camera.height,self.camera.width],
            [self.camera.height,self.camera.width],self.camera,self.mount,max_segment_m=1.)
        if len(points)<2:
            raise ValueError("cannot determine calibrated reference start: "+reason)
        self.start_x=float(points[0,0])
        if not .05<self.start_x<=1.5:
            raise ValueError("calibrated near reference must be within 1.5 m; inspect physical field of view")
        self.controller=ControllerConfig(cruise_speed_mps=.05,max_angular_speed_rps=.2,
            max_path_offset_m=1.5,lookahead_m=.5)
        self.limiter=CommandLimiter(self.controller)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.goal=self.route=self.pending=self.reference=self.pose=None
        self.completed=False
        self.source_ns=0;self.source_received=float("-inf")
        self.reason="waiting_for_goal";self.detail={};self.images=OrderedDict()
        self.last_tick=time.monotonic();self.last_overlay_stamp=0
        self.last_seg_ns=0;self.last_display=None;self.overlay_pending=None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.path_pub=self.create_publisher(Path,"/husky_nav/vision_reference",1)
        self.command_pub=self.create_publisher(TwistStamped,"/husky_nav/vision_cmd_vel",1)
        self.status_pub=self.create_publisher(String,"/husky_nav/vision_control_status",1)
        self.overlay_pub=self.create_publisher(CompressedImage,"/husky_nav/vision_overlay/compressed",sensor)
        self.subs=[self.create_subscription(String,"/drivable_area/segmentation_packet",self.segmentation,1),
            self.create_subscription(PoseStamped,"/husky_nav/goal",self.new_goal,qos),
            self.create_subscription(Path,"/husky_nav/goal_plan",self.new_route,qos),
            self.create_subscription(Odometry,"/husky_link/odom",self.odometry,sensor),
            self.create_subscription(Image,value("image_topic"),self.image,sensor)]
        self.cancel_service=self.create_service(Trigger,"/husky_nav/cancel_vision_goal",self.cancel)
        self.retry_timer=self.create_timer(.02,self.retry)
        self.timer=self.create_timer(.05,self.tick)
        cv2.setNumThreads(1)

    def tf_matrix(self,target,source,ns):
        tf=self.buffer.lookup_transform(target,source,Time(nanoseconds=ns,clock_type=self.get_clock().clock_type))
        q,t=tf.transform.rotation,tf.transform.translation
        matrix=np.eye(4);matrix[:3,:3]=quaternion_matrix(q.x,q.y,q.z,q.w)
        matrix[:3,3]=[t.x,t.y,t.z]
        if (not np.isfinite(matrix).all() or abs(matrix[2,2]-1)>1e-3):
            raise ValueError("nonplanar_or_invalid_capture_tf")
        return matrix

    def clear(self,reason):
        self.reference=None;self.reason=reason;self.detail={};self.overlay_pending=None
        path=Path();path.header.frame_id=self.odom_frame
        path.header.stamp=self.get_clock().now().to_msg();self.path_pub.publish(path)
        self.limiter.reset()
        self.publish_command(0.,0.)
        if reason in ("goal_cancelled","waiting_for_goal_guide","goal_reached") and self.last_display is not None:
            ns,labels=self.last_display
            self.overlay(ns,labels,[],reason)

    def new_goal(self,message):
        self.goal=None;self.route=None;self.pending=None
        self.completed=False
        self.clear("waiting_for_goal_guide")
        p=message.pose.position
        if message.header.frame_id!="map" or not all(math.isfinite(v) for v in (p.x,p.y,p.z)):
            self.reason="invalid_goal";return
        self.goal=message

    def new_route(self,message):
        self.route=None
        try:
            if self.goal is None or message.header.frame_id!="map":
                raise ValueError("waiting_for_goal_guide")
            if any(p.header.frame_id not in ("","map") or not math.isfinite(p.pose.position.z)
                   for p in message.poses):
                raise ValueError("invalid_guide_frames")
            points=validate_path([[p.pose.position.x,p.pose.position.y] for p in message.poses],.5)
            p=self.goal.pose.position
            if stamp_ns(message.header.stamp)<stamp_ns(self.goal.header.stamp) or np.linalg.norm(points[-1]-[p.x,p.y])>.2:
                raise ValueError("guide_does_not_match_goal")
            self.route=(message,points,time.monotonic())
        except (ValueError,TypeError):
            self.clear("waiting_for_goal_guide")

    def cancel(self,request,response):
        self.goal=self.route=self.pending=None
        self.clear("goal_cancelled")
        response.success=True;response.message="vision goal cancelled; zero request"
        return response

    def odometry(self,message):
        self.pose=None
        try:
            if message.header.frame_id!=self.odom_frame or message.child_frame_id!=self.base:
                return
            p,q=message.pose.pose.position,message.pose.pose.orientation
            rotation=quaternion_matrix(q.x,q.y,q.z,q.w)
            pose=np.array([p.x,p.y,math.atan2(rotation[1,0],rotation[0,0])])
            if np.isfinite(pose).all():
                self.pose=(pose,stamp_ns(message.header.stamp),time.monotonic())
        except (ValueError,TypeError):
            pass

    def image(self,message):
        if (message.header.frame_id!=self.mount.camera_frame or
                (message.height,message.width)!=(self.camera.height,self.camera.width)):
            return
        if self.overlay_pub.get_subscription_count()==0:
            self.images.clear();return
        self.images[stamp_ns(message.header.stamp)]=message
        while len(self.images)>12:
            self.images.popitem(last=False)
        ns=stamp_ns(message.header.stamp)
        pending=self.overlay_pending
        if pending is not None and pending[0]==ns:
            self.overlay_pending=None
            self.overlay(*pending)
        if ns-self.last_seg_ns>500_000_000 or ns-self.last_overlay_stamp>500_000_000:
            self.overlay(ns,np.full((self.camera.height,self.camera.width),255,np.uint8),[],"waiting for fresh AI")

    def segmentation(self,message):
        self.pending=None
        try:
            packet,labels=decode_segmentation(message.data)
            source=packet["source_header"];stamp=source["stamp"]
            if (source["frame_id"]!=self.mount.camera_frame or
                    packet["raw_shape_hw"]!=[self.camera.height,self.camera.width] or
                    type(stamp["sec"]) is not int or type(stamp["nanosec"]) is not int or
                    not 0<=stamp["nanosec"]<10**9):
                raise ValueError("invalid_capture_geometry")
            ns=stamp["sec"]*10**9+stamp["nanosec"]
            now=time.monotonic()
            if not fresh_stamp(ns,now,self.get_clock().now().nanoseconds,now,.5):
                raise ValueError("stale_segmentation")
            self.last_seg_ns=ns
            self.pending=(ns,labels,now)
            self.retry()
        except (ValueError,TypeError,KeyError,OverflowError,cv2.error) as exc:
            self.clear(str(exc)[:120])

    def retry(self):
        if self.pending is None:
            return
        ns,labels,received=self.pending
        if not fresh_stamp(ns,received,self.get_clock().now().nanoseconds,time.monotonic(),.5):
            self.pending=None;self.clear("capture_expired_waiting_for_tf");return
        if self.completed:
            self.pending=None;self.clear("goal_reached")
            self.overlay(ns,labels,[],self.reason);return
        if self.goal is None or self.route is None:
            self.pending=None;self.clear("waiting_for_goal_guide")
            self.overlay(ns,labels,[],self.reason);return
        try:
            route,points,_=self.route
            age=(self.get_clock().now().nanoseconds-stamp_ns(route.header.stamp))/1e9
            if not -.05<=age<=5:
                raise ValueError("global_guide_stale")
            ground_from_map=self.tf_matrix(self.mount.ground_frame,"map",ns)
            odom_from_ground=self.tf_matrix(self.odom_frame,self.mount.ground_frame,ns)
            mapped=points@ground_from_map[:2,:2].T+ground_from_map[:2,3]
            p=self.goal.pose.position
            goal=np.array([p.x,p.y])@ground_from_map[:2,:2].T+ground_from_map[:2,3]
            target=route_direction(mapped,goal)
            sample=lambda p:sample_ground_labels(p,labels,self.camera,self.mount)
            reference,reason,detail=plan_vision_reference(sample,self.start_x,target,self.config)
            if not len(reference):
                raise ValueError(reason)
            if not fresh_stamp(ns,received,self.get_clock().now().nanoseconds,time.monotonic(),.5):
                raise ValueError("vision_planning_too_old")
            metric=reference@odom_from_ground[:2,:2].T+odom_from_ground[:2,3]
            self.reference=metric;self.source_ns=ns;self.source_received=received
            self.reason="tracking_image_reference";self.detail=detail
            path=Path();path.header.frame_id=self.odom_frame
            path.header.stamp.sec,path.header.stamp.nanosec=divmod(ns,10**9)
            for x,y in metric:
                pose=PoseStamped();pose.header=path.header
                pose.pose.position.x=float(x);pose.pose.position.y=float(y);pose.pose.orientation.w=1.
                path.poses.append(pose)
            self.path_pub.publish(path)
            self.pending=None
            self.overlay(ns,labels,reference,"image reference ready")
        except TransformException:
            # One bounded latest packet, retried at its original capture time.
            self.clear("waiting_for_capture_tf")
        except (ValueError,TypeError,KeyError,OverflowError,cv2.error) as exc:
            self.pending=None;self.clear(str(exc)[:120])
            self.overlay(ns,labels,[],self.reason)

    def overlay(self,ns,labels,reference,reason):
        message=self.images.get(ns)
        if self.overlay_pub.get_subscription_count()==0:
            self.overlay_pending=None
            return
        if message is None:
            # Image and tiny segmentation messages can arrive in either order.
            # One slot only; never paint the reference on a different capture.
            self.overlay_pending=(ns,labels,reference,reason)
            return
        if (self.get_clock().now().nanoseconds-ns)/1e9>.5:
            return
        try:
            if message.header.frame_id!=self.mount.camera_frame:
                return
            frame=image_to_bgr(message)
            image=draw_vision_reference(frame,labels,reference,self.camera,self.mount,reason)
            result=fill_compressed_image(CompressedImage(),image,message.header,80)
            self.overlay_pub.publish(result);self.last_overlay_stamp=ns
            self.last_display=(ns,labels)
        except (ValueError,TypeError,cv2.error,OSError) as exc:
            self.get_logger().warning("Vision overlay: "+str(exc)[:120],throttle_duration_sec=2.)

    def publish_command(self,v,w):
        message=TwistStamped();message.header.frame_id=self.base
        message.header.stamp=self.get_clock().now().to_msg()
        message.twist.linear.x=float(v);message.twist.angular.z=float(w)
        self.command_pub.publish(message)

    def demand(self):
        now=time.monotonic();ns=self.get_clock().now().nanoseconds
        if self.goal is None:
            return Demand(reason="waiting_for_goal")
        if self.completed:
            return Demand(reason="goal_reached")
        if self.pose is None or not fresh_stamp(self.pose[1],self.pose[2],ns,now,.3):
            return Demand(reason="odometry_invalid_or_stale")
        try:
            matrix=self.tf_matrix(self.odom_frame,"map",self.pose[1])
        except TransformException:
            return Demand(reason="waiting_for_current_goal_tf")
        p=self.goal.pose.position
        goal=np.array([p.x,p.y])@matrix[:2,:2].T+matrix[:2,3]
        distance=float(np.linalg.norm(goal-self.pose[0][:2]))
        if distance<=.15:
            return Demand(reason="goal_reached",remaining_m=distance)
        if self.reference is None:
            return Demand(reason=self.reason)
        if not fresh_stamp(self.source_ns,self.source_received,ns,now,.5):
            return Demand(reason="image_reference_stale")
        demand=pursuit(self.reference,self.pose[0],self.controller)
        if demand.reason!="tracking":
            return demand
        delta=goal-self.pose[0][:2];yaw=self.pose[0][2]
        if float(delta@[math.cos(yaw),math.sin(yaw)])<=0:
            return Demand(reason="goal_not_ahead")
        speed=min(demand.linear,math.sqrt(2*self.controller.max_deceleration_mps2*max(distance-.15,0)))
        scale=speed/max(demand.linear,1e-9)
        return Demand(speed,demand.angular*scale,"tracking",demand.cross_track_m,distance,demand.target_robot_xy)

    def tick(self):
        now=time.monotonic();dt=max(1e-6,now-self.last_tick);self.last_tick=now
        try:
            demand=self.demand()
        except (ValueError,TypeError,OverflowError):
            demand=Demand(reason="invalid_vision_control_inputs")
        v,w=self.limiter.step(demand,dt)
        self.publish_command(v,w)
        self.status_pub.publish(String(data=json.dumps(dict(reason=demand.reason,
            reference_reason=self.reason,source_stamp_ns=self.source_ns,linear_x=v,angular_z=w,
            remaining_to_goal_m=demand.remaining_m,controller_path_topic="/husky_nav/vision_reference",
            steering_source="image_ai",map_use="goal_route_guidance",**self.detail,
            driver_output=False,ready_for_autonomous_control=False),allow_nan=False)))
        if demand.reason=="image_reference_stale" and self.reference is not None:
            self.clear(demand.reason)
        if demand.reason=="goal_reached" and not self.completed:
            self.completed=True;self.pending=None;self.clear("goal_reached")


def main(args=None):
    rclpy.init(args=args);node=VisionGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_command(0.,0.)
        node.destroy_node();rclpy.try_shutdown()
