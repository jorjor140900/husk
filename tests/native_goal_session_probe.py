"""ROS transport/closed-loop synthetic probe: run only in a network-none container.

Requires the exported husky_vision package. This publishes private preview
requests and synthetic ROS topics, so never run alongside a real robot graph.
"""
import os
if os.environ.get("HUSKY_ISOLATED_TEST") != "1":
    raise SystemExit("Use HUSKY_ISOLATED_TEST=1 in an isolated --network none ROS test container only.")
import json, math, time
from collections import deque
from pathlib import Path
import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from husky_vision.safety_node import LaserSafetyNode
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import PoseStamped,TransformStamped,Twist,TwistStamped
from nav_msgs.msg import Path as RosPath,Odometry
from sensor_msgs.msg import Image,CompressedImage
from std_msgs.msg import String,Bool,Header
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from husky_vision.vision_goal_node import VisionGoalNode
from husky_vision.navigation_gate import NavigationGate
from husky_vision.ground_geometry import CameraCalibration,CameraMount
from husky_vision.transport import fill_image
from husky_vision.vision_grid import encode_segmentation
from road_navigation.detector import DetectorConfig,RoadDetector
out=Path('/out');out.mkdir(exist_ok=True)
camera=dict(image_width=640,image_height=480,distortion_model='plumb_bob',camera_matrix=dict(data=[500.,0.,319.5,0.,500.,239.5,0.,0.,1.]),distortion_coefficients=dict(data=[0.]*5))
mount=dict(height_m=.99,pitch_down_deg=20.,roll_deg=0.,yaw_deg=0.,camera_x_m=-.26,camera_y_m=.21,ground_frame='vision_ground',camera_frame='camera_optical_frame')
(out/'camera.yaml').write_text(yaml.safe_dump(camera));(out/'mount.json').write_text(json.dumps(mount))
rclpy.init(args=['--ros-args','-p','camera_calibration:='+str(out/'camera.yaml'),'-p','camera_mount:='+str(out/'mount.json'),'-p','guide_only:=true','-p','supervised_trial:=true','-p','trial_goal_session:=true'])
vision=VisionGoalNode()
gate=NavigationGate(parameter_overrides=[Parameter('allow_motion',value=True),Parameter('supervised_trial',value=True)],
 cli_args=['--ros-args','-r','/husky_nav/nav2_cmd_vel:=/husky_nav/vision_cmd_vel',
 '-r','/husky_nav/vision_status:=/husky_nav/vision_control_status'])
safety=LaserSafetyNode(namespace='husky_control',parameter_overrides=[Parameter('supervised_trial',value=True),Parameter('trial_goal_session',value=True)],
 cli_args=['--ros-args','-r','odom:=/husky_link/odom','-r','motion_permitted:=/husky_control/motion_permitted',
 '-r','control/requested_cmd_vel:=/husky_nav/guarded_request'])
fixture=Node('isolated_reference_gate_fixture')
executor=SingleThreadedExecutor()
for n in (vision,gate,safety,fixture):executor.add_node(n)
sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
image_pub=fixture.create_publisher(Image,'/camera/image_raw',sensor)
seg_pub=fixture.create_publisher(String,'/drivable_area/segmentation_packet',1)
odom_pub=fixture.create_publisher(Odometry,'/husky_link/odom',sensor)
goal_pub=fixture.create_publisher(PoseStamped,'/husky_nav/goal',latched)
route_pub=fixture.create_publisher(RosPath,'/husky_nav/goal_plan',latched)
permission_pub=fixture.create_publisher(Bool,'/husky_control/motion_permitted',1)
legacy_pub=fixture.create_publisher(String,'/husky_nav/vision_status',1)
current_goal=None;last_route=0.
static=StaticTransformBroadcaster(fixture);dynamic=TransformBroadcaster(fixture)
def tf(parent,child,x=0.,z=0.):
 t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.transform.translation.x=x;t.transform.translation.z=z;t.transform.rotation.w=1.;return t
static.sendTransform([tf('map','odom'),tf('base_link','vision_ground',z=-.132)])
tf_delay=float(os.environ.get('HUSKY_TEST_TF_DELAY_SEC','0'))
assert 0<=tf_delay<=.3
tf_pending=deque()
state=dict(x=0.,y=0.,yaw=0.,v=0.,w=0.,vision=True,blocked=False,simulate=False,permission=True)
requests=[];guarded=[];references=[];statuses=[];images=[];action_count=[];filtered=[];gate_status=[];filter_status=[]
filtered_motion=[0.,0.]
def receive_filtered(m):
 filtered_motion[:]=[m.twist.linear.x,m.twist.angular.z]
 filtered.append(tuple(filtered_motion))
subs_extra=[fixture.create_subscription(TwistStamped,'/husky_control/control/cmd_vel_stamped',
 receive_filtered,1),
 fixture.create_subscription(String,'/husky_nav/control_status',lambda m:gate_status.append(json.loads(m.data)),1),
 fixture.create_subscription(String,'/husky_control/safety/status',lambda m:filter_status.append(json.loads(m.data)),1)]
def command(m):
 state['v']=m.twist.linear.x;state['w']=m.twist.angular.z;requests.append((time.monotonic(),state['v'],state['w']))
def overlay(m):
 image=cv2.imdecode(np.frombuffer(m.data,np.uint8),cv2.IMREAD_COLOR)
 assert image.shape==(480,640,3)
 yellow=(image[90:,:,0]<80)&(image[90:,:,1]>180)&(image[90:,:,2]>180)
 images.append((m.header.stamp.sec*10**9+m.header.stamp.nanosec,int(yellow.sum()),image))
subs=[fixture.create_subscription(TwistStamped,'/husky_nav/vision_cmd_vel',command,1),fixture.create_subscription(Twist,'/husky_nav/guarded_request',lambda m:guarded.append((m.linear.x,m.angular.z)),1),fixture.create_subscription(RosPath,'/husky_nav/vision_reference',lambda m:references.append(m),1),fixture.create_subscription(String,'/husky_nav/vision_control_status',lambda m:statuses.append(json.loads(m.data)),1),fixture.create_subscription(CompressedImage,'/husky_nav/vision_overlay/compressed',overlay,sensor)]
def publish_route():
 global last_route
 if current_goal is None:return
 path=RosPath();path.header.frame_id='map';path.header.stamp=fixture.get_clock().now().to_msg()
 goal=current_goal.pose.position
 for t in np.linspace(0,1,101):
  p=PoseStamped();p.header=path.header;p.pose.position.x=state['x']+(goal.x-state['x'])*t;p.pose.position.y=state['y']+(goal.y-state['y'])*t+.25*math.sin(math.pi*t);p.pose.orientation.w=1.;path.poses.append(p)
 route_pub.publish(path);last_route=time.monotonic();action_count.append(1)
c=CameraCalibration.from_mapping(camera);m=CameraMount.from_mapping(mount)
yy,xx=np.mgrid[:480,:640];raw=np.column_stack([xx.ravel(),yy.ravel()]).astype(float)
normalized=cv2.undistortPoints(raw.reshape(-1,1,2),c.matrix,c.distortion).reshape(-1,2)
rays=np.column_stack([normalized,np.ones(len(raw))])@m.rotation.T
valid=rays[:,2]<-1e-6
hit=m.origin-(m.height_m/np.minimum(rays[:,2],-1e-6))[:,None]*rays
probability=((np.abs(hit[:,1])<.6)&valid).reshape(480,640).astype(float)
frame=np.full((480,640,3),(30,120,40),np.uint8);frame[probability>0]=(140,140,140)
detector=RoadDetector(DetectorConfig(generate_reference_path=False,use_temporal=False,reject_green_vegetation=True))
clear=detector.process(frame,drivable_probability=probability)
blocked=detector.process(frame,drivable_probability=np.zeros((480,640)))
last=time.monotonic()
def publish():
 global last
 now=time.monotonic();dt=min(now-last,.15);last=now
 if state['simulate']:
  state['x']+=filtered_motion[0]*math.cos(state['yaw'])*dt;state['y']+=filtered_motion[0]*math.sin(state['yaw'])*dt;state['yaw']+=filtered_motion[1]*dt
 stamp=fixture.get_clock().now().to_msg()
 permission_pub.publish(Bool(data=state['permission']))
 legacy_pub.publish(String(data=json.dumps(dict(valid=False,reason='waiting_for_capture_tf',
  source_stamp_ns=stamp.sec*10**9+stamp.nanosec))))
 odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_link';odom.header.stamp=stamp
 odom.pose.pose.position.x=state['x'];odom.pose.pose.position.y=state['y'];odom.pose.pose.orientation.z=math.sin(state['yaw']/2);odom.pose.pose.orientation.w=math.cos(state['yaw']/2);odom.twist.twist.linear.x=state['v'];odom.twist.twist.angular.z=state['w'];odom_pub.publish(odom)
 tr=tf('odom','base_link');tr.header.stamp=stamp;tr.transform.translation.x=state['x'];tr.transform.translation.y=state['y'];tr.transform.rotation=odom.pose.pose.orientation;tf_pending.append((time.monotonic()+tf_delay,tr))
 header=Header();header.frame_id=m.camera_frame;header.stamp=stamp
 image_pub.publish(fill_image(Image(),frame,header))
 if time.monotonic()-last_route>1.:publish_route()
 if state['vision']:seg_pub.publish(String(data=encode_segmentation(blocked if state['blocked'] else clear,header,.3)))
def publish_ready_tf():
 while tf_pending and tf_pending[0][0]<=time.monotonic():
  dynamic.sendTransform(tf_pending.popleft()[1])
tf_timer=fixture.create_timer(.01,publish_ready_tf)
timer=fixture.create_timer(1/15,publish)
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:executor.spin_once(timeout_sec=.01)
def goal(x,y=0.):
 global current_goal
 p=PoseStamped();p.header.frame_id='map';p.header.stamp=fixture.get_clock().now().to_msg();p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;current_goal=p;goal_pub.publish(p);publish_route()
try:
 spin(.8);gate.enabled=True;state['simulate']=True;goal(1.4)
 started=time.monotonic();end=started+45.
 while time.monotonic()<end:
  executor.spin_once(timeout_sec=.01)
  if statuses and statuses[-1]['reason']=='goal_reached':break
 elapsed=time.monotonic()-started
 assert statuses[-1]['reason']=='goal_reached',(state,statuses[-5:],filter_status[-5:])
 spin(.3)
 assert state['x']>.5,state
 assert elapsed>20.,elapsed
 assert max(v for v,w in filtered)>.04
 assert max(v for v,w in filtered)<=.050001
 assert all(v==0 and w==0 for v,w in filtered[-4:])
 assert math.hypot(state['x']-1.4,state['y'])<=.16
 assert fixture.count_publishers('/cmd_vel')==0
 assert safety.trial.limits['max_distance_m']==3.
 assert safety.trial.limits['max_duration_sec']==120.
 report=dict(passed=True,physical_robot=False,synthetic_camera=True,
  goal_reached=True,goal_error_m=math.hypot(state['x']-1.4,state['y']),traveled_x_m=state['x'],
  elapsed_sec=elapsed,passed_old_half_meter_and_twenty_second_limits=True,
  max_filtered_linear_mps=max(v for v,w in filtered),stopped_at_goal=True,
  delayed_tf_sec=tf_delay,legacy_grid_invalid_throughout=True,no_global_cmd_vel=True)
 (out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
finally:
 executor.shutdown()
 for n in (vision,gate,safety,fixture):n.destroy_node()
 rclpy.try_shutdown()
