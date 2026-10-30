"""Run only in a network-none ROS container; all inputs/operator are synthetic."""
import os
if os.environ.get('HUSKY_ISOLATED_TEST')!='1':raise SystemExit('isolated test only')
import json,time,math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy
from geometry_msgs.msg import TwistStamped,TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String,Bool
from std_srvs.srv import SetBool,Trigger
from tf2_ros import StaticTransformBroadcaster,TransformBroadcaster
from husky_vision.navigation_gate import NavigationGate
from husky_vision.safety_node import LaserSafetyNode
rclpy.init()
nav=NavigationGate(parameter_overrides=[Parameter('allow_motion',value=True),Parameter('supervised_trial',value=True)],
 cli_args=['--ros-args','-r','/husky_nav/nav2_cmd_vel:=/husky_nav/vision_cmd_vel'])
def args():
 return ['--ros-args','-r','odom:=/husky_link/odom','-r','scan:=/husky_link/scan',
 '-r','motion_permitted:=/husky_control/motion_permitted','-r','control/requested_cmd_vel:=/husky_nav/guarded_request']
safety=LaserSafetyNode(namespace='husky_control',cli_args=args(),parameter_overrides=[Parameter('supervised_trial',value=True),Parameter('release_hold_sec',value=.05)])
normal=LaserSafetyNode(namespace='normal_trial_test',cli_args=args())
f=Node('isolated_supervised_fixture');ex=SingleThreadedExecutor()
for node in (nav,safety,normal,f):ex.add_node(node)
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
pubs={name:f.create_publisher(cls,topic,q) for name,cls,topic,q in [
 ('odom',Odometry,'/husky_link/odom',qos),('scan',LaserScan,'/husky_link/scan',qos),
 ('vision',String,'/husky_nav/vision_status',1),('command',TwistStamped,'/husky_nav/vision_cmd_vel',1),
 ('permission',Bool,'/husky_control/motion_permitted',1)]}
state=dict(x=0.,y=0.,yaw=0.,v=0.,w=0.,vision=True,odom=True,permission=False)
commands=[];statuses=[];normal_commands=[]
def receive(m):
 state['v'],state['w']=m.twist.linear.x,m.twist.angular.z
 commands.append((time.monotonic(),state['v'],state['w']))
subs=[f.create_subscription(TwistStamped,'/husky_control/control/cmd_vel_stamped',receive,1),
 f.create_subscription(TwistStamped,'/normal_trial_test/control/cmd_vel_stamped',lambda m:normal_commands.append((m.twist.linear.x,m.twist.angular.z)),1),
 f.create_subscription(String,'/husky_control/safety/status',lambda m:statuses.append(json.loads(m.data)),1)]
static=StaticTransformBroadcaster(f);dynamic=TransformBroadcaster(f)
t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom';t.transform.rotation.w=1.;static.sendTransform(t)
last=time.monotonic()
def publish():
 global last
 now=time.monotonic();dt=now-last;last=now
 state['x']+=state['v']*math.cos(state['yaw'])*dt;state['y']+=state['v']*math.sin(state['yaw'])*dt;state['yaw']+=state['w']*dt
 stamp=f.get_clock().now().to_msg();ns=stamp.sec*10**9+stamp.nanosec
 if state['odom']:
  m=Odometry();m.header.frame_id='odom';m.child_frame_id='base_link';m.header.stamp=stamp
  m.pose.pose.position.x=state['x'];m.pose.pose.position.y=state['y']
  m.pose.pose.orientation.z=math.sin(state['yaw']/2);m.pose.pose.orientation.w=math.cos(state['yaw']/2)
  m.twist.twist.linear.x=state['v'];m.twist.twist.angular.z=state['w'];pubs['odom'].publish(m)
  t=TransformStamped();t.header.frame_id='odom';t.child_frame_id='base_link';t.header.stamp=stamp
  t.transform.translation.x=state['x'];t.transform.translation.y=state['y'];t.transform.rotation=m.pose.pose.orientation;dynamic.sendTransform(t)
 m=TwistStamped();m.header.frame_id='base_link';m.header.stamp=stamp;m.twist.linear.x=.05;pubs['command'].publish(m)
 pubs['vision'].publish(String(data=json.dumps(dict(valid=state['vision'],source_stamp_ns=ns))))
 pubs['permission'].publish(Bool(data=state['permission']))
 scan=LaserScan();scan.header.frame_id='velodyne';scan.header.stamp=stamp
 scan.angle_min=-math.pi;scan.angle_max=math.pi;scan.angle_increment=.007;scan.range_min=0.;scan.range_max=200.;scan.ranges=[float('inf')]*897
 pubs['scan'].publish(scan)
timer=f.create_timer(.05,publish)
def spin(duration):
 end=time.monotonic()+duration
 while time.monotonic()<end:ex.spin_once(timeout_sec=.01)
def call(name,kind,request):
 client=f.create_client(kind,name);assert client.wait_for_service(timeout_sec=2.)
 future=client.call_async(request)
 end=time.monotonic()+2
 while not future.done() and time.monotonic()<end:ex.spin_once(timeout_sec=.01)
 assert future.done(),name
 result=future.result();f.destroy_client(client);return result
def stopped():return commands and all(v==0 and w==0 for _,v,w in commands[-4:])
def restart():
 state['permission']=False;spin(.4)
 assert call('/husky_nav/reset_trial',Trigger,Trigger.Request()).success
 state['permission']=True;spin(.8)
 assert any(v>.005 for _,v,w in commands[-8:]),statuses[-5:]
try:
 spin(.6)
 assert call('/husky_nav/set_enabled',SetBool,SetBool.Request(data=True)).success
 state['permission']=True;spin(.9)
 assert any(v>.005 for _,v,w in commands),statuses[-8:]
 assert max(v for _,v,w in commands)<=.030001
 assert all(v==0 and w==0 for v,w in normal_commands),'Normal mode passed invalid scan'
 assert statuses[-1]['lidar_checked'] is False
 assert not call('/husky_nav/reset_trial',Trigger,Trigger.Request()).success
 state['vision']=False;spin(.35);assert stopped(),statuses[-5:]
 state['vision']=True;spin(.4);assert stopped(),'Vision recovery resumed a latched trial'
 restart();state['odom']=False;spin(.6);assert stopped(),statuses[-5:]
 state['odom']=True;spin(.4);assert stopped(),'Odometry recovery resumed a latched trial'
 restart();start_x=state['x']
 spin(18)
 assert stopped(),statuses[-5:]
 assert statuses[-1]['reason']=='trial_distance_limit',statuses[-5:]
 assert state['x']-start_x<.52
 restart();safety.trial.started=time.monotonic()-21.;spin(.2);assert stopped()
 assert statuses[-1]['reason']=='trial_time_limit'
 assert f.count_publishers('/cmd_vel')==0
 report=dict(passed=True,physical_robot=False,synthetic_inputs_and_operator=True,
  trial_moves_with_explicit_lidar_bypass=True,normal_mode_rejects_same_invalid_scan=True,
  vision_loss_stops_and_latches=True,odometry_loss_stops_and_latches=True,
  reset_rejected_while_permission_true=True,distance_limit_stops=True,time_limit_stops=True,
  no_global_cmd_vel=True,max_speed_mps=max(v for _,v,w in commands))
 Path('/out/native-report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
finally:
 ex.shutdown()
 for node in (nav,safety,normal,f):node.destroy_node()
 rclpy.try_shutdown()
