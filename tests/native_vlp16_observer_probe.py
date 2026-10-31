import os
if os.environ.get('HUSKY_ISOLATED_TEST')!='1':raise SystemExit('isolated container only')
import json,time
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from husky_vision.robot_bridge_node import RobotBridgeNode
from husky_vision.robot_link import RosbridgeLink
from husky_vision.vlp16_observer import Vlp16Observer
from husky_link_core.cloud_wire import encode_cloud
class Stamp:
 __slots__=('secs','nsecs')
 def __init__(self,ns):self.secs,self.nsecs=divmod(ns,10**9)
class Header:
 __slots__=('stamp','frame_id')
 def __init__(self,ns):self.stamp,self.frame_id=Stamp(ns),'velodyne'
rclpy.init(args=['--ros-args','-r','__ns:=/husky_link','-p','input_mode:=raw_timed'])
with patch.object(RosbridgeLink,'start',lambda self:None):bridge=RobotBridgeNode()
observer=Vlp16Observer();fixture=Node('isolated_cloud_fixture')
executor=SingleThreadedExecutor()
for node in (bridge,observer,fixture):executor.add_node(node)
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
odom=fixture.create_publisher(Odometry,'/husky_link/odom',qos)
tf=StaticTransformBroadcaster(fixture)
extrinsic=TransformStamped();extrinsic.header.frame_id='base_link';extrinsic.child_frame_id='velodyne'
extrinsic.transform.rotation.w=1.;extrinsic.transform.translation.z=.88;tf.sendTransform(extrinsic)
outputs=[];statuses=[];raw=[];emitted=set()
def output(message):
 points=np.frombuffer(message.data,dtype='<f4').reshape(-1,3)
 assert message.header.frame_id=='base_link'
 assert message.header.stamp.sec*10**9+message.header.stamp.nanosec in emitted
 np.testing.assert_allclose(points,[[5,0,.88],[5,0,.88]],atol=.002)
 outputs.append(message)
subs=[fixture.create_subscription(PointCloud2,'/husky_nav/vlp16_observed_points',output,qos),
 fixture.create_subscription(PointCloud2,'/husky_link/points',lambda m:raw.append(m),qos),
 fixture.create_subscription(String,'/husky_nav/vlp16_status',lambda m:statuses.append(json.loads(m.data)),1)]
start=fixture.get_clock().now().nanoseconds;last_odom=last_cloud=0
last_packet=None
try:
 end=time.monotonic()+4
 while time.monotonic()<end:
  now=fixture.get_clock().now().nanoseconds
  if now-start>1_000_000_000 and now-last_odom>20_000_000:
   m=Odometry();m.header.frame_id='odom';m.child_frame_id='base_link'
   m.header.stamp=fixture.get_clock().now().to_msg();m.pose.pose.orientation.w=1.
   m.pose.pose.position.x=(now-start)/1e9;m.twist.twist.linear.x=1.;odom.publish(m);last_odom=now
  if now-start>100_000_000 and now-last_cloud>200_000_000:
   source=now-30_000_000
   dtype=np.dtype(dict(names=['x','y','z','ring','time'],formats=['<f4','<f4','<f4','<u2','<f4'],offsets=[0,4,8,12,14],itemsize=18))
   a=np.zeros(2,dtype=dtype);a['x']=[5.1,5];a['time']=[-.1,0];a['ring']=[0,15]
   fields=[dict(name=n,offset=o,datatype=d,count=1) for n,o,d in [('x',0,7),('y',4,7),('z',8,7),('ring',12,4),('time',14,7)]]
   m=NS(header=Header(source),height=1,width=2,point_step=18,row_step=36,is_bigendian=False,is_dense=True,fields=fields,data=a.tobytes())
   last_packet=dict(protocol=1,kind='cloud',msg=encode_cloud(m));emitted.add(source)
   bridge.link._received('cloud',last_packet);last_cloud=now
  executor.spin_once(timeout_sec=.005)
 assert any(s['reason']=='waiting_for_bracketing_odometry' for s in statuses)
 assert len(outputs)>5,(len(outputs),statuses[-6:])
 assert raw[-1].header.frame_id=='velodyne'
 assert all(s['motion_safety_validated'] is False and s['control_output'] is False for s in statuses)
 # Repeated delivery of an old capture must not renew source freshness.
 end=time.monotonic()+.7
 while time.monotonic()<end:
  bridge.link._received('cloud',last_packet)
  executor.spin_once(timeout_sec=.01)
 assert statuses[-1]['valid'] is False,statuses[-5:]
 assert statuses[-1]['reason']=='cloud_timeout',statuses[-5:]
 assert fixture.count_publishers('/cmd_vel')==0
 report=dict(passed=True,synthetic_cloud=True,physical_robot=False,outputs=len(outputs),
  raw_fields_and_timing_preserved=True,same_stamp_base_frame_deskew=True,
  stale_retransmission_stays_invalid=True,no_global_velocity_publisher=True,
  live_ros1_or_websocket_tested=False,coverage_validated=False)
 Path('/out/native-report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
finally:
 executor.shutdown()
 bridge.destroy_node();observer.destroy_node();fixture.destroy_node();rclpy.try_shutdown()
