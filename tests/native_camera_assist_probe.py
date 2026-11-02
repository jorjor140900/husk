"""Synthetic ROS2 assistance regression. Requires a network-isolated container."""
import os
from pathlib import Path
if os.environ.get('HUSKY_ISOLATED_TEST')!='1' or set(p.name for p in Path('/sys/class/net').iterdir())-{'lo'}:
    raise SystemExit('Only run in a network-isolated test container')
import json,math,time,subprocess,sys
from collections import deque
from types import SimpleNamespace as NS
import cv2,numpy as np,yaml,rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy
from geometry_msgs.msg import Twist,TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image,CompressedImage
from std_msgs.msg import Bool,Header,String
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from husky_vision.safety_node import LaserSafetyNode
from husky_vision.ground_geometry import CameraCalibration,CameraMount
from husky_vision.transport import fill_image
from husky_vision.vision_grid import encode_segmentation

out=Path('/results/camera-assist-validation');out.mkdir(exist_ok=True)
camera=dict(image_width=640,image_height=480,distortion_model='plumb_bob',camera_matrix=dict(data=[500.,0.,319.5,0.,500.,239.5,0.,0.,1.]),distortion_coefficients=dict(data=[0.]*5))
mount=dict(height_m=.99,pitch_down_deg=20.,roll_deg=0.,yaw_deg=0.,camera_x_m=-.26,camera_y_m=.21,ground_frame='vision_ground',camera_frame='camera_optical_frame')
(out/'camera.yaml').write_text(yaml.safe_dump(camera));(out/'mount.json').write_text(json.dumps(mount))
args=['--ros-args','-p','camera_calibration:='+str(out/'camera.yaml'),'-p','camera_mount:='+str(out/'mount.json'),
    '-p','allow_motion:=true','-p','supervised_trial:=true','-p','trial_goal_session:=true',
    '-p','trial_extended_session:=true','-p','trial_extended_duration_sec:=900.0','-p','trial_turning_session:=true',
    '-p','pause_on_vision_gap:=true','-p','subscribe_scan:=false','-p','half_width_m:=0.33','-p','side_margin_m:=0.08',
    '-r','laser_safety:odom:=/husky_link/odom',
    '-r','laser_safety:control/requested_cmd_vel:=/husky_assist/guarded_request',
    '-r','laser_safety:/husky_nav/control_status:=/husky_assist/gate_status']
rclpy.init(args=args)
process=subprocess.Popen([sys.executable,'-c','from husky_vision.camera_assist_node import main; main()']+args)
fixture=Node('camera_assist_fixture');safety=LaserSafetyNode(namespace='husky_control')
executor=SingleThreadedExecutor();executor.add_node(fixture);executor.add_node(safety)
sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
image_pub=fixture.create_publisher(Image,'/camera/image_raw',sensor)
seg_pub=fixture.create_publisher(String,'/drivable_area/segmentation_packet',1)
odom_pub=fixture.create_publisher(Odometry,'/husky_link/odom',sensor)
permission=fixture.create_publisher(Bool,'/husky_control/motion_permitted',1)
teleop=fixture.create_publisher(Twist,'/husky_assist/teleop_cmd_vel',1)
static=StaticTransformBroadcaster(fixture);dynamic=TransformBroadcaster(fixture)
t=TransformStamped();t.header.frame_id='base_link';t.child_frame_id='vision_ground';t.transform.rotation.w=1.;static.sendTransform(t)
state=dict(ai=True,blocked=False,held=False,sign=0.,x=0.,y=0.,yaw=0.,v=0.,w=0.,simulate=False)
statuses=[];outputs=[];overlays=[];tf_pending=deque()
def output(msg):
    outputs.append((msg.linear.x,msg.angular.z));state['v']=msg.linear.x;state['w']=msg.angular.z
subs=[fixture.create_subscription(String,'/husky_assist/status',lambda m:statuses.append(json.loads(m.data)),1),
    fixture.create_subscription(Twist,'/husky_control/control/cmd_vel',output,1),
    fixture.create_subscription(CompressedImage,'/husky_assist/overlay/compressed',lambda m:overlays.append(m),sensor)]
c=CameraCalibration.from_mapping(camera);m=CameraMount.from_mapping(mount)
yy,xx=np.mgrid[:480,:640];pixels=np.column_stack([xx.ravel(),yy.ravel()]).astype(float)
normal=cv2.undistortPoints(pixels.reshape(-1,1,2),c.matrix,c.distortion).reshape(-1,2)
rays=np.column_stack([normal,np.ones(len(pixels))])@m.rotation.T
valid=rays[:,2]<-1e-6
hit=m.origin-(m.height_m/np.minimum(rays[:,2],-1e-6))[:,None]*rays
last=time.monotonic()
def publish():
    global last
    now=time.monotonic();dt=min(.1,now-last);last=now
    if state['simulate']:
        state['x']+=state['v']*math.cos(state['yaw'])*dt;state['y']+=state['v']*math.sin(state['yaw'])*dt;state['yaw']+=state['w']*dt
    stamp=fixture.get_clock().now().to_msg()
    odom=Odometry();odom.header=Header(frame_id='odom',stamp=stamp);odom.child_frame_id='base_link'
    odom.pose.pose.position.x=state['x'];odom.pose.pose.position.y=state['y'];odom.pose.pose.orientation.z=math.sin(state['yaw']/2);odom.pose.pose.orientation.w=math.cos(state['yaw']/2)
    odom.twist.twist.linear.x=state['v'];odom.twist.twist.angular.z=state['w'];odom_pub.publish(odom)
    t=TransformStamped();t.header=odom.header;t.child_frame_id='base_link';t.transform.translation.x=state['x'];t.transform.translation.y=state['y'];t.transform.rotation=odom.pose.pose.orientation
    tf_pending.append((now+.09,t))
    permission.publish(Bool(data=True));command=Twist();command.linear.x=.15 if state['held'] else 0.;teleop.publish(command)
    co,si=math.cos(state['yaw']),math.sin(state['yaw'])
    wx=state['x']+co*hit[:,0]-si*hit[:,1];wy=state['y']+si*hit[:,0]+co*hit[:,1]
    centre=state['sign']*.22*np.maximum(wx-.7,0)**2
    mask=(valid&(abs(wy-centre)<.95)).reshape(480,640).astype(np.uint8)*255
    if state['blocked']:mask[:]=0
    header=Header(frame_id=m.camera_frame,stamp=stamp)
    frame=np.full((480,640,3),60,np.uint8);frame[mask>0]=140
    image_pub.publish(fill_image(Image(),frame,header))
    if state['ai']:seg_pub.publish(String(data=encode_segmentation(NS(raw_mask=mask,input_shape=(480,640)),header,.3)))
def tf_ready():
    while tf_pending and tf_pending[0][0]<=time.monotonic():dynamic.sendTransform(tf_pending.popleft()[1])
fixture.create_timer(1/15,publish);fixture.create_timer(.01,tf_ready)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
def stopped():return len(outputs)>=4 and all(v==0 and w==0 for v,w in outputs[-4:])
try:
    spin(1.5)
    client=fixture.create_client(SetBool,'/husky_assist/set_enabled');assert client.wait_for_service(timeout_sec=2.)
    future=client.call_async(SetBool.Request(data=True));spin(.4);assert future.done() and future.result().success
    assert stopped()  # Enabled alone cannot drive.
    state['held']=True;spin(1.2);assert any(v>0 for v,w in outputs[-10:]),statuses[-2:]
    budget=safety.trial.started
    state['held']=False;spin(.3);assert stopped();assert not safety.trial.fault
    for sign in (-1.,1.):
        state['sign']=sign;state['held']=True;spin(1.2)
        assert any(sign*s['angular_z']>.003 for s in statuses[-10:]),statuses[-2:]
    state['simulate']=True;spin(2.);assert state['x']>.05
    state['ai']=False;spin(.9)
    assert statuses[-1]['using_held_reference'] and 0<statuses[-1]['linear_x']<=.05,statuses[-1]
    spin(1.3);assert stopped();assert not safety.trial.fault
    state['ai']=True;spin(1.2);assert any(v>0 for v,w in outputs[-10:]),statuses[-2:]
    state['blocked']=True;spin(.6);assert stopped();assert not safety.trial.fault
    state['blocked']=False;spin(1.2);assert any(v>0 for v,w in outputs[-10:]),statuses[-2:]
    state['held']=False;spin(.3);assert stopped()
    assert safety.trial.started==budget and not safety.trial.fault
    assert fixture.count_publishers('/cmd_vel')==0
    assert fixture.count_publishers('/scan')==0 and fixture.count_publishers('/goal_pose')==0
    assert safety.scan_sub is None
    assert overlays
    cv2.imwrite(str(out/'overlay.png'),cv2.imdecode(np.frombuffer(overlays[-1].data,np.uint8),cv2.IMREAD_COLOR))
    report=dict(result='passed',native_ros2=True,synthetic=True,physical_robot=False,
        held_forward_required=True,release_stops=True,left_right_curves_steered=True,
        missing_image_expires=True,new_veto_stops=True,recovery_without_latched_fault=True,
        session_budget_not_reset=True,no_goal_map_lidar_or_global_cmd_vel=True,simulated_distance_x_m=state['x'])
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
finally:
    process.terminate()
    try:process.wait(timeout=5.)
    except subprocess.TimeoutExpired:process.kill();process.wait()
    executor.shutdown();fixture.destroy_node();safety.destroy_node();rclpy.try_shutdown()
