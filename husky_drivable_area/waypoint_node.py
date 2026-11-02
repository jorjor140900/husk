"""RViz goal clicks build a queue; explicit start dispatches one goal at a time."""
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .waypoint_queue import WaypointQueue


class WaypointNode(Node):
    def __init__(self):
        super().__init__('waypoint_queue')
        self.queue=WaypointQueue();self.last_token=0;self.reason='collecting_waypoints';self.cancelling=False
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.goal_pub=self.create_publisher(PoseStamped,'/husky_nav/waypoint_goal',qos)
        self.path_pub=self.create_publisher(Path,'/husky_nav/waypoints',qos)
        self.status_pub=self.create_publisher(String,'/husky_nav/waypoints/status',1)
        self.subs=[self.create_subscription(PoseStamped,'/goal_pose',self.add,10),
            self.create_subscription(String,'/husky_nav/vision_control_status',self.control_status,10)]
        self.queue_services=[self.create_service(Trigger,'/husky_nav/waypoints/start',self.start),
            self.create_service(Trigger,'/husky_nav/waypoints/clear',self.clear)]
        self.cancel_client=self.create_client(Trigger,'/husky_nav/cancel_vision_goal')
        self.timer=self.create_timer(.5,self.publish)

    def add(self,msg):
        p=msg.pose.position
        if msg.header.frame_id!='map' or not all(math.isfinite(v) for v in (p.x,p.y,p.z)) or abs(p.z)>.01:
            self.reason='waypoint_requires_map_xy';self.publish();return
        try:self.queue.add(p.x,p.y)
        except ValueError as exc:self.reason=str(exc)
        self.publish()

    def dispatch(self):
        token=max(self.get_clock().now().nanoseconds,self.last_token+1)
        self.last_token=token;self.queue.token=token
        msg=PoseStamped();msg.header.frame_id='map'
        msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(token,10**9)
        msg.pose.position.x,msg.pose.position.y=self.queue.points[self.queue.index]
        msg.pose.orientation.w=1.
        self.reason='following_waypoint';self.goal_pub.publish(msg);self.publish()

    def start(self,request,response):
        if self.cancelling:
            response.success=False;response.message='cancel pending';return response
        try:self.queue.start()
        except ValueError as exc:
            response.success=False;response.message=str(exc);return response
        self.dispatch();response.success=True;response.message='queue started; operator permission still required'
        return response

    def control_status(self,msg):
        try:
            status=json.loads(msg.data)
            if status.get('reason')!='goal_reached':return
            remaining=status.get('remaining_to_goal_m')
            if type(remaining) not in (int,float) or not 0<=remaining<=.15:return
            if not self.queue.reached(status.get('goal_stamp_ns')):return
        except (ValueError,TypeError,AttributeError):return
        if self.queue.active:self.dispatch()
        else:self.reason='queue_completed';self.publish()

    def clear(self,request,response):
        if not self.cancel_client.service_is_ready():
            response.success=False;response.message='vision cancel service unavailable';return response
        self.queue.active=False;self.cancelling=True;self.reason='cancelling_queue';self.publish()
        future=self.cancel_client.call_async(Trigger.Request())
        def finished(result):
            try:
                if result.result().success:self.queue.clear();self.reason='queue_cleared'
                else:self.reason='queue_cancel_failed'
            except Exception:self.reason='queue_cancel_failed'
            self.cancelling=False
            self.publish()
        future.add_done_callback(finished)
        response.success=True;response.message='cancel requested; wait for queue_cleared status'
        return response

    def publish(self):
        path=Path();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
        for x,y in self.queue.points:
            p=PoseStamped();p.header=path.header;p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.
            path.poses.append(p)
        self.path_pub.publish(path)
        self.status_pub.publish(String(data=json.dumps(dict(reason=self.reason,active=self.queue.active,
            total=len(self.queue.points),current_index=self.queue.index,goal_stamp_ns=self.queue.token,
            waypoints_xy=self.queue.points))))


def main(args=None):
    rclpy.init(args=args);node=WaypointNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
