"""Native ROS goal/image/scan integration. ONLY in an isolated test container.

Synthetic scan-plane rays, segmentation, TF and odometry; no robot or Nav2
planner. The preview gate stays closed, even during the kinematic simulation.
"""
import os
from pathlib import Path
if os.environ.get('HUSKY_ISOLATED_TEST')!='1' or set(p.name for p in Path('/sys/class/net').iterdir())-{'lo'}:
    raise SystemExit('Run only with HUSKY_ISOLATED_TEST=1 in a network-isolated ROS container.')

from collections import deque
import json,math,time
from types import SimpleNamespace as NS
import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import PoseStamped,TransformStamped,Twist,TwistStamped
from nav_msgs.msg import Odometry,Path as RosPath,OccupancyGrid
from sensor_msgs.msg import Image,LaserScan,CompressedImage
from std_msgs.msg import Header,String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from husky_vision.vision_goal_node import VisionGoalNode
from husky_vision.goal_dispatch import GoalDispatch
from husky_vision.navigation_gate import NavigationGate
from husky_vision.ground_geometry import CameraCalibration,CameraMount
from husky_vision.transport import fill_image
from husky_vision.vision_grid import encode_segmentation
from husky_vision.collision import circle_scan

def room_scan(pose,circles):
    count=900;x,y,yaw=pose
    angle=-math.pi+np.arange(count)*2*math.pi/count+yaw
    directions=np.column_stack([np.cos(angle),np.sin(angle)])
    sides=np.where(directions>=0,10.,-10.)
    with np.errstate(divide='ignore',invalid='ignore'):
        distances=(sides-[x,y])/directions
    walls=np.min(np.where(distances>0,distances,np.inf),axis=1)
    ranges=np.minimum(walls,circle_scan(pose,circles,count=count,range_max=50.))
    return NS(ranges=ranges,angle_min=-math.pi,angle_increment=2*math.pi/count,
        angle_max=-math.pi+(count-1)*2*math.pi/count,range_min=.1,range_max=50.,scan_time=0.,time_increment=0.)

out=Path('/results/local-fusion-validation/native');out.mkdir(exist_ok=True)
camera=dict(image_width=640,image_height=480,distortion_model='plumb_bob',
    camera_matrix=dict(data=[500.,0.,319.5,0.,500.,239.5,0.,0.,1.]),distortion_coefficients=dict(data=[0.]*5))
mount=dict(height_m=.99,pitch_down_deg=20.,roll_deg=0.,yaw_deg=0.,camera_x_m=-.26,camera_y_m=.21,
    ground_frame='vision_ground',camera_frame='camera_optical_frame')
(out/'camera.yaml').write_text(yaml.safe_dump(camera));(out/'mount.json').write_text(json.dumps(mount))
rclpy.init(args=['--ros-args','-p','camera_calibration:='+str(out/'camera.yaml'),'-p','camera_mount:='+str(out/'mount.json'),
    '-p','guidance_mode:=local_fusion','-p','guide_only:=true','-p','local_guidance:=true',
    '-p','supervised_trial:=true','-p','trial_goal_session:=true','-p','trial_extended_session:=true',
    '-p','trial_extended_duration_sec:=900.0','-p','allow_motion:=false',
    # This ideal fixture sensor observes from 0.1m. The field VLP16 keeps 0.4m.
    '-p','lidar_min_range_m:=0.1'])
vision=VisionGoalNode();dispatch=GoalDispatch();gate=NavigationGate();fixture=Node('isolated_local_fusion_fixture')
executor=SingleThreadedExecutor()
for node in (vision,dispatch,gate,fixture):executor.add_node(node)
sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
image_pub=fixture.create_publisher(Image,'/camera/image_raw',sensor)
seg_pub=fixture.create_publisher(String,'/drivable_area/segmentation_packet',1)
scan_pub=fixture.create_publisher(LaserScan,'/husky_link/scan',sensor)
odom_pub=fixture.create_publisher(Odometry,'/husky_link/odom',sensor)
goal_pub=fixture.create_publisher(PoseStamped,'/goal_pose',1)
empty_route_pub=fixture.create_publisher(RosPath,'/husky_nav/goal_plan',latched)
static=StaticTransformBroadcaster(fixture);dynamic=TransformBroadcaster(fixture)
def transform(parent,child,x=0.,z=0.):
    t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
    t.transform.translation.x=x;t.transform.translation.z=z;t.transform.rotation.w=1.;return t
static.sendTransform([transform('map','odom'),transform('base_link','vision_ground',z=-.132),
    transform('base_link','laser',x=-.2317,z=.88155)])
state=dict(x=0.,y=0.,yaw=0.,v=0.,w=0.,ai=True,lidar=True,unknown=False,obstacle=False,blocked=False,simulate=False)
commands=[];guarded=[];statuses=[];references=[];overlays=[];grids=[];tf_pending=deque()
def command(message):
    state['v']=message.twist.linear.x;state['w']=message.twist.angular.z
    commands.append((state['v'],state['w']))
def overlay(message):
    frame=cv2.imdecode(np.frombuffer(message.data,np.uint8),cv2.IMREAD_COLOR)
    yellow=(frame[90:,:,0]<80)&(frame[90:,:,1]>180)&(frame[90:,:,2]>180)
    overlays.append((message.header.stamp.sec*10**9+message.header.stamp.nanosec,int(yellow.sum()),frame))
subs=[fixture.create_subscription(TwistStamped,'/husky_nav/vision_cmd_vel',command,1),
    fixture.create_subscription(Twist,'/husky_nav/guarded_request',lambda m:guarded.append((m.linear.x,m.angular.z)),1),
    fixture.create_subscription(String,'/husky_nav/vision_control_status',lambda m:statuses.append(json.loads(m.data)),1),
    fixture.create_subscription(RosPath,'/husky_nav/vision_reference',lambda m:references.append(m),1),
    fixture.create_subscription(CompressedImage,'/husky_nav/vision_overlay/compressed',overlay,sensor),
    fixture.create_subscription(OccupancyGrid,'/husky_nav/local_fusion_grid',lambda m:grids.append(m),1)]
c=CameraCalibration.from_mapping(camera);m=CameraMount.from_mapping(mount)
yy,xx=np.mgrid[:480,:640];pixels=np.column_stack([xx.ravel(),yy.ravel()]).astype(float)
normalized=cv2.undistortPoints(pixels.reshape(-1,1,2),c.matrix,c.distortion).reshape(-1,2)
rays=np.column_stack([normalized,np.ones(len(pixels))])@m.rotation.T
valid=rays[:,2]<-1e-6
hit=m.origin-(m.height_m/np.minimum(rays[:,2],-1e-6))[:,None]*rays
previous=time.monotonic();last_empty=0.
def publish():
    global previous,last_empty
    now=time.monotonic();dt=min(now-previous,.15);previous=now
    if state['simulate']:
        state['x']+=state['v']*math.cos(state['yaw'])*dt;state['y']+=state['v']*math.sin(state['yaw'])*dt;state['yaw']+=state['w']*dt
    stamp=fixture.get_clock().now().to_msg();yaw=state['yaw'];co,si=math.cos(yaw),math.sin(yaw)
    odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_link';odom.header.stamp=stamp
    odom.pose.pose.position.x=state['x'];odom.pose.pose.position.y=state['y']
    odom.pose.pose.orientation.z=math.sin(yaw/2);odom.pose.pose.orientation.w=math.cos(yaw/2)
    odom.twist.twist.linear.x=state['v'];odom.twist.twist.angular.z=state['w'];odom_pub.publish(odom)
    t=transform('odom','base_link');t.header.stamp=stamp;t.transform.translation.x=state['x'];t.transform.translation.y=state['y'];t.transform.rotation=odom.pose.pose.orientation
    tf_pending.append((time.monotonic()+.09,t))
    if state['lidar']:
        circles=[(state['x']+.85*co,state['y']+.85*si,.15)] if state['obstacle'] else []
        raw=room_scan([state['x']-.2317*co,state['y']-.2317*si,yaw],circles)
        scan=LaserScan();scan.header.frame_id='laser';scan.header.stamp=stamp
        for name in ('angle_min','angle_max','angle_increment','range_min','range_max','time_increment','scan_time'):
            setattr(scan,name,float(getattr(raw,name)))
        scan.ranges=(np.full(len(raw.ranges),np.inf) if state['unknown'] else raw.ranges).astype(np.float32).tolist();scan_pub.publish(scan)
    world_y=state['y']+si*hit[:,0]+co*hit[:,1]
    mask=(valid&(np.abs(world_y)<1.2)).reshape(480,640).astype(np.uint8)*255
    if state['blocked']:mask[:]=0
    frame=np.full((480,640,3),55,np.uint8);frame[mask>0]=145
    header=Header(frame_id=m.camera_frame,stamp=stamp);image_pub.publish(fill_image(Image(),frame,header))
    if state['ai']:
        result=NS(raw_mask=mask,input_shape=(480,640))
        seg_pub.publish(String(data=encode_segmentation(result,header,.3)))
    if now-last_empty>1.:
        empty=RosPath();empty.header.frame_id='map';empty.header.stamp=stamp
        empty_route_pub.publish(empty);last_empty=now
def ready_tf():
    while tf_pending and tf_pending[0][0]<=time.monotonic():dynamic.sendTransform(tf_pending.popleft()[1])
timer=fixture.create_timer(1/15,publish);tf_timer=fixture.create_timer(.01,ready_tf)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
def goal(x):
    message=PoseStamped();message.header.frame_id='map';message.header.stamp=fixture.get_clock().now().to_msg()
    message.pose.position.x=x;message.pose.orientation.w=1.;goal_pub.publish(message)
def stopped():return len(commands)>=4 and all(v==0 and w==0 for v,w in commands[-4:])
try:
    spin(1.5);goal(4.);spin(2.)
    assert dispatch.client is None and vision.route is None
    assert any(v>.01 for v,w in commands),statuses[-5:]
    assert any(len(p.poses)>20 for p in references)
    assert grids and set(grids[-1].data)<={-1,0,100}
    assert any(n>100 for ns,n,frame in overlays)
    path_stamps={p.header.stamp.sec*10**9+p.header.stamp.nanosec for p in references if p.poses}
    assert any(ns in path_stamps and n>100 for ns,n,frame in overlays)
    cv2.imwrite(str(out/'controller-overlay.jpg'),next(frame for ns,n,frame in reversed(overlays) if n>100))
    state['obstacle']=True;spin(.8);assert stopped(),statuses[-4:]
    state['obstacle']=False;spin(.8);assert any(v>.01 for v,w in commands[-5:])
    before=len(commands);state['lidar']=False;spin(.7);assert stopped(),statuses[-4:]
    assert len(commands)-before>=3,'Controller stopped publishing instead of emitting zero commands'
    assert fixture.get_clock().now().nanoseconds-vision.last_seg_ns<500_000_000,'AI also went stale in missing-LiDAR case'
    state['lidar']=True;state['unknown']=True;spin(.7);assert stopped()
    state['unknown']=False;spin(.8);assert any(v>.01 for v,w in commands[-5:])
    state['blocked']=True;spin(.7);assert stopped()
    state['blocked']=False;spin(.8);state['ai']=False;spin(.8);assert stopped()
    state['ai']=True;spin(.8)
    cancel=fixture.create_client(Trigger,'/husky_nav/cancel_vision_goal');assert cancel.wait_for_service(timeout_sec=1.)
    result=cancel.call_async(Trigger.Request());spin(.4);assert result.done() and result.result().success and stopped()
    state['simulate']=True;goal(2.)
    deadline=time.monotonic()+25
    while time.monotonic()<deadline:
        executor.spin_once(timeout_sec=.005)
        if statuses and statuses[-1]['reason']=='goal_reached':break
    assert statuses[-1]['reason']=='goal_reached',(state,statuses[-6:])
    error=math.hypot(state['x']-2.,state['y']);assert error<=.15
    assert guarded and all(v==0 and w==0 for v,w in guarded)
    assert fixture.count_publishers('/cmd_vel')==0
    report=dict(result='passed',native_ros2=True,synthetic=True,physical_robot=False,
        no_global_planner_or_action_client=True,empty_global_route_does_not_clear_local_reference=True,
        actual_yellow_matches_capture_and_controller_reference=True,live_obstacle_stops=True,
        missing_lidar_stops=True,unknown_lidar_stops=True,image_veto_stops=True,stale_image_stops=True,
        cancel_stops=True,delayed_tf_sec=.09,synthetic_goal_reached=True,synthetic_goal_distance_m=2.,
        synthetic_sensor_min_range_m=.1,
        goal_error_m=error,planning_only_gate_stays_zero=True,no_global_cmd_vel=True,
        note='Ideal synthetic snapshot scan rays, image labels, odometry and TF. No physical/terrain/height validation.')
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
finally:
    executor.shutdown()
    for node in (vision,dispatch,gate,fixture):node.destroy_node()
    rclpy.try_shutdown()
