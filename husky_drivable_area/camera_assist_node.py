"""Hold-forward teleoperation with camera-area steering; no goals or map."""
from collections import OrderedDict
import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile,ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image,CompressedImage
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer,TransformListener,TransformException

from .area_reference import plan_area_reference
from .control import ControllerConfig,Demand,fresh_stamp,pursuit,quaternion_matrix
from .controller_node import stamp_ns
from .ground_geometry import CameraCalibration,CameraMount,project_path
from .ground_overlay import reference_seed
from .reference_stability import reusable_reference
from .transport import image_to_bgr,fill_compressed_image
from .vision_goal import VisionReferenceConfig,sample_ground_labels
from .vision_goal_overlay import draw_vision_reference
from .vision_grid import decode_segmentation
from road_navigation.detector import DetectorConfig


class CameraAssistNode(Node):
    def __init__(self):
        super().__init__('camera_assist')
        defaults=dict(camera_calibration='',camera_mount='',base_frame='base_link',odom_frame='odom',
            image_topic='/camera/image_raw',allow_motion=False,speed_mps=.15,max_angular_rps=.15,
            half_width_m=.33,side_margin_m=.08,reference_hold_sec=2.)
        for key,value in defaults.items():self.declare_parameter(key,value)
        value=lambda key:self.get_parameter(key).value
        self.allow_motion=value('allow_motion');self.enabled=False
        self.hold=value('reference_hold_sec')
        if not .5<=self.hold<=2.:raise ValueError('reference_hold_sec must be within 0.5..2 seconds')
        speed,angular=value('speed_mps'),value('max_angular_rps')
        if not 0<speed<=.15 or not 0<angular<=.15:raise ValueError('assist speed limits exceed supervised preset')
        self.camera=CameraCalibration.load(value('camera_calibration'))
        self.mount=CameraMount.load(value('camera_mount'))
        self.base,self.odom_frame=value('base_frame'),value('odom_frame')
        self.config=VisionReferenceConfig(half_width_m=value('half_width_m'),side_margin_m=value('side_margin_m'))
        seed=reference_seed(self.camera,self.mount,DetectorConfig(max_width=max(640,self.camera.width)))
        u,v=seed['seed_xy']
        points,reason=project_path([[u,v],[u,v-1]],[self.camera.height,self.camera.width],
            [self.camera.height,self.camera.width],self.camera,self.mount,max_segment_m=1.)
        if len(points)<2 or not .05<points[0,0]<=1.5:raise ValueError('camera near field not calibrated: '+reason)
        self.start_x=float(points[0,0])
        self.controller=ControllerConfig(cruise_speed_mps=speed,max_angular_speed_rps=angular,
            max_path_offset_m=1.5,lookahead_m=.5)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.teleop=None;self.pose=None;self.pending=None;self.reference=None
        self.source_ns=0;self.source_received=float('-inf');self.reason='waiting_for_image'
        self.detail={};self.images=OrderedDict();self.display=None;self.capture_plans=OrderedDict()
        sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.output=self.create_publisher(Twist,'/husky_assist/guarded_request',1)
        self.status=self.create_publisher(String,'/husky_assist/status',1)
        self.gate=self.create_publisher(String,'/husky_assist/gate_status',1)
        self.overlay_pub=self.create_publisher(CompressedImage,'/husky_assist/overlay/compressed',sensor)
        self.subs=[self.create_subscription(Twist,'/husky_assist/teleop_cmd_vel',self.receive_teleop,1),
            self.create_subscription(String,'/drivable_area/segmentation_packet',self.segmentation,1),
            self.create_subscription(Odometry,'/husky_link/odom',self.odometry,sensor),
            self.create_subscription(Image,value('image_topic'),self.image,sensor)]
        self.service=self.create_service(SetBool,'/husky_assist/set_enabled',self.enable)
        self.retry_timer=self.create_timer(.02,self.retry)
        self.timer=self.create_timer(.05,self.tick)
        cv2.setNumThreads(1)

    def enable(self,request,response):
        self.enabled=bool(request.data and self.allow_motion)
        self.teleop=None  # Enabling cannot replay a previously held key.
        self.output.publish(Twist())
        response.success=not request.data or self.allow_motion
        response.message='enabled; fresh held-forward input and robot permission required' if self.enabled else 'disabled / preview'
        return response

    def receive_teleop(self,message):
        self.teleop=(message,time.monotonic())
        if message.linear.x<=0:self.output.publish(Twist())

    def odometry(self,message):
        self.pose=None
        try:
            if message.header.frame_id!=self.odom_frame or message.child_frame_id!=self.base:return
            p,q=message.pose.pose.position,message.pose.pose.orientation
            rotation=quaternion_matrix(q.x,q.y,q.z,q.w)
            pose=np.array([p.x,p.y,math.atan2(rotation[1,0],rotation[0,0])])
            if not np.isfinite(pose).all() or abs(rotation[2,2]-1)>1e-3:return
            self.pose=(pose,stamp_ns(message.header.stamp),time.monotonic())
        except (ValueError,TypeError,OverflowError):pass

    def fresh_reference(self):
        return self.reference is not None and fresh_stamp(self.source_ns,self.source_received,
            self.get_clock().now().nanoseconds,time.monotonic(),self.hold)

    def clear(self,reason):
        self.reference=None;self.pending=None;self.reason=reason;self.detail={}
        self.output.publish(Twist())

    def tf_matrix(self,ns):
        tf=self.buffer.lookup_transform(self.odom_frame,self.mount.ground_frame,
            Time(nanoseconds=ns,clock_type=self.get_clock().clock_type))
        q,t=tf.transform.rotation,tf.transform.translation
        matrix=np.eye(4);matrix[:3,:3]=quaternion_matrix(q.x,q.y,q.z,q.w)
        matrix[:3,3]=[t.x,t.y,t.z]
        if not np.isfinite(matrix).all() or abs(matrix[2,2]-1)>1e-3:raise ValueError('invalid_capture_tf')
        return matrix

    def segmentation(self,message):
        try:
            packet=json.loads(message.data)
            if isinstance(packet,dict) and packet.get('valid') is False and packet.get('reason') in (
                    'camera_timeout','stale_input','stale_result','stale_output','stale_segmentation_before_preview'):
                reason=packet['reason']
                if not self.fresh_reference():self.clear(reason)
                return
            packet,labels=decode_segmentation(message.data)
            source=packet['source_header'];stamp=source['stamp']
            if (source['frame_id']!=self.mount.camera_frame or packet['raw_shape_hw']!=[self.camera.height,self.camera.width]
                    or type(stamp['sec']) is not int or type(stamp['nanosec']) is not int or not 0<=stamp['nanosec']<10**9):
                raise ValueError('invalid_capture_geometry')
            ns=stamp['sec']*10**9+stamp['nanosec'];now=time.monotonic()
            if not fresh_stamp(ns,now,self.get_clock().now().nanoseconds,now,.5):
                if not self.fresh_reference():self.clear('stale_segmentation')
                return
            if ns<=self.source_ns:return
            path,reason,_=self.plan_capture(ns,labels)
            caption=('preview only' if not self.allow_motion else 'preview; robot TF required') if len(path) else reason
            self.display=(ns,labels,path,caption)
            if not self.allow_motion:
                self.render();self.reason='preview_only';return
            if self.pending is None or not fresh_stamp(self.pending[0],self.pending[2],
                    self.get_clock().now().nanoseconds,now,.5):self.pending=(ns,labels,now)
            self.retry()
        except (ValueError,TypeError,KeyError,OverflowError,cv2.error):self.clear('invalid_segmentation_packet')

    def plan_capture(self,ns,labels):
        if not hasattr(self,'capture_plans'):self.capture_plans=OrderedDict()
        if ns not in self.capture_plans:
            sample=lambda points:sample_ground_labels(points,labels,self.camera,self.mount)
            self.capture_plans[ns]=plan_area_reference(sample,self.start_x,config=self.config)
            while len(self.capture_plans)>12:self.capture_plans.popitem(last=False)
        return self.capture_plans[ns]

    def retry(self):
        if self.pending is None:return
        ns,labels,received=self.pending
        if not fresh_stamp(ns,received,self.get_clock().now().nanoseconds,time.monotonic(),.5):
            self.pending=None
            if not self.fresh_reference():self.clear('capture_expired_waiting_for_tf')
            return
        try:
            matrix=self.tf_matrix(ns)
            sample=lambda points:sample_ground_labels(points,labels,self.camera,self.mount)
            path,reason,detail=self.plan_capture(ns,labels)
            old=None
            if self.fresh_reference():
                old=(self.reference-matrix[:2,3])@matrix[:2,:2]
                previous=reusable_reference(old,sample,self.start_x,self.config.half_width_m+self.config.side_margin_m)
                if previous is not None and (not len(path) or np.linalg.norm(previous[-1]-path[-1])<.15):
                    path=previous;reason='ok';detail={**self.detail,'reference_reused_after_current_mask_check':True}
            self.pending=None
            if not len(path) and old is not None and self.fresh_reference():
                # An unseen strip can temporarily retain the original odom
                # reference. Known non-drivable pixels still reject it.
                held=reusable_reference(old,sample,self.start_x,
                    self.config.half_width_m+self.config.side_margin_m,allow_unknown=True)
                if held is not None:
                    self.detail.update(control_reference_held=True,holding_reference_reason=reason)
                    self.display=(ns,labels,held,'HELD CONTROL; waiting for new reference')
                    self.render();return  # Do not renew source_ns/source_received.
            if not len(path):
                self.clear(reason);self.detail=detail;self.display=(ns,labels,np.empty((0,2)),reason);self.render();return
            if not fresh_stamp(ns,received,self.get_clock().now().nanoseconds,time.monotonic(),.5):
                if not self.fresh_reference():self.clear('vision_planning_too_old')
                return
            self.reference=path@matrix[:2,:2].T+matrix[:2,3]
            self.source_ns=ns;self.source_received=received;self.reason='tracking';self.detail={**detail,'control_reference_held':False}
            self.last_good_world=self.reference.copy();self.last_good_world_stamp=ns
            self.display=(ns,labels,path,'ready; hold forward in teleop');self.render()
        except TransformException:
            self.render()
            if not self.fresh_reference():self.reason='waiting_for_capture_tf'
        except (ValueError,TypeError,KeyError,OverflowError,cv2.error):self.clear('invalid_image_geometry')

    def demand(self):
        now=time.monotonic();ns=self.get_clock().now().nanoseconds
        if not self.allow_motion:return Demand(reason='preview_only')
        if not self.enabled:return Demand(reason='disabled')
        if self.teleop is None or now-self.teleop[1]>.2:return Demand(reason='teleop_released_or_stale')
        t=self.teleop[0]
        values=(t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z)
        if not all(math.isfinite(x) for x in values) or any(x!=0 for x in (t.linear.y,t.linear.z,t.angular.x,t.angular.y)):
            return Demand(reason='invalid_teleop')
        if t.linear.x<=0:return Demand(reason='teleop_released_or_stale')
        if self.pose is None or not fresh_stamp(self.pose[1],self.pose[2],ns,now,.3):return Demand(reason='odometry_invalid_or_stale')
        if self.reference is None:return Demand(reason=self.reason)
        if not self.fresh_reference():return Demand(reason='image_reference_stale')
        demand=pursuit(self.reference,self.pose[0],self.controller)
        if demand.reason=='goal_reached':return Demand(reason='reference_exhausted')
        if demand.reason!='tracking':return demand
        speed=min(t.linear.x,demand.linear,self.controller.max_angular_speed_rps/max(float(self.detail.get('max_curvature',0)),1e-9))
        if self.detail.get('control_reference_held') or (ns-self.source_ns)/1e9>.5:speed=min(speed,.05)
        return Demand(speed,demand.angular*speed/max(demand.linear,1e-9),'tracking')

    def tick(self):
        try:demand=self.demand()
        except (ValueError,TypeError,OverflowError):demand=Demand(reason='invalid_assist_inputs')
        command=Twist();command.linear.x=demand.linear;command.angular.z=demand.angular
        self.output.publish(command)
        idle=demand.reason in ('teleop_released_or_stale','invalid_teleop')
        gate_open=self.allow_motion and self.enabled and (demand.reason=='tracking' or idle)
        self.gate.publish(String(data=json.dumps(dict(enabled=self.enabled,allow_motion=self.allow_motion,
            request_allowed=gate_open,reason='request_passed_to_collision_filter' if gate_open else 'vision_invalid',
            vision_reason=demand.reason))))
        age=(self.get_clock().now().nanoseconds-self.source_ns)/1e6 if self.source_ns else None
        self.status.publish(String(data=json.dumps(dict(self.detail,mode='camera_steering_assist',reason=demand.reason,
            enabled=self.enabled,allow_motion=self.allow_motion,linear_x=demand.linear,angular_z=demand.angular,
            source_stamp_ns=self.source_ns,reference_age_ms=age,reference_valid=self.fresh_reference(),using_held_reference=self.fresh_reference() and (bool(self.detail.get('control_reference_held')) or age is not None and age>500),
            reference_hold_sec=self.hold,goal_required=False,lidar_required=False,map_required=False,
            red_edges_used_for_planning=False),allow_nan=False)))

    def image(self,message):
        if message.header.frame_id!=self.mount.camera_frame or (message.height,message.width)!=(self.camera.height,self.camera.width):return
        if not self.overlay_pub.get_subscription_count():self.images.clear();return
        ns=stamp_ns(message.header.stamp);self.images[ns]=message
        while len(self.images)>24:self.images.popitem(last=False)
        if self.display is not None and self.display[0]==ns:self.render()

    def render(self):
        if self.display is None or not self.overlay_pub.get_subscription_count():return
        ns,labels,path,reason=self.display;message=self.images.get(ns)
        if message is None:return
        try:
            if len(path)>=2 and not reason.startswith('HELD'):
                self.last_good_overlay=(ns,np.array(path,copy=True))
            elif len(path)<2 and getattr(self,'last_good_overlay',None) is not None:
                held_ns,path=self.last_good_overlay
                path=path.copy()
                if getattr(self,'last_good_world_stamp',None)==held_ns:
                    try:
                        matrix=self.tf_matrix(ns)
                        path=(self.last_good_world-matrix[:2,3])@matrix[:2,:2]
                    except (TransformException,ValueError,TypeError):pass
                age=max(0.,(ns-held_ns)/1e9)
                reason=f'HELD DISPLAY ONLY {age:.1f}s | STOP: '+reason
            frame=draw_vision_reference(image_to_bgr(message),labels,path,self.camera,self.mount,reason,
                edges=[],assisted=True)
            self.overlay_pub.publish(fill_compressed_image(CompressedImage(),frame,message.header,80))
        except (ValueError,TypeError,cv2.error):pass


def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=CameraAssistNode();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if node is not None:
            if rclpy.ok():node.output.publish(Twist())
            node.destroy_node()
        rclpy.try_shutdown()
