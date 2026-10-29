"""RViz /goal_pose -> native Nav2 actions, planning-only by default."""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from std_msgs.msg import String


class GoalDispatch(Node):
    def __init__(self):
        super().__init__("goal_dispatch")
        self.declare_parameter("allow_motion", False)
        self.allow_motion = self.get_parameter("allow_motion").value
        self.action_type = NavigateToPose if self.allow_motion else ComputePathToPose
        self.client = ActionClient(self,self.action_type,"/navigate_to_pose" if self.allow_motion else "/compute_path_to_pose")
        self.subscription = self.create_subscription(PoseStamped,"/goal_pose",self.goal,1)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path,"/husky_nav/goal_plan",latched)
        self.goal_pub = self.create_publisher(PoseStamped,"/husky_nav/goal",latched)
        self.status = self.create_publisher(String,"/husky_nav/goal_status",1)
        self.generation, self.goal_handle = 0, None
        self.status_message = None
        self.status_timer = self.create_timer(1., self.publish_status)
        self.report("waiting_for_goal")

    def report(self, reason, **extra):
        self.status_message = String(data=json.dumps(dict(reason=reason,planning_only=not self.allow_motion,**extra)))
        self.publish_status()

    def publish_status(self):
        if self.status_message is not None:
            self.status.publish(self.status_message)

    def goal(self, message):
        p,q = message.pose.position,message.pose.orientation
        values = (p.x,p.y,p.z,q.x,q.y,q.z,q.w)
        if (message.header.frame_id != "map" or not all(math.isfinite(v) for v in values) or
                abs(p.z)>.01 or abs(q.x)>1e-3 or abs(q.y)>1e-3 or abs(q.z*q.z+q.w*q.w-1)>1e-3):
            self.report("goal_requires_finite_map_pose")
            return
        self.generation += 1
        generation = self.generation
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        empty = Path()
        empty.header = message.header
        self.path_pub.publish(empty)
        self.goal_pub.publish(message)
        if not self.client.server_is_ready():
            self.report("action_server_not_ready_set_initial_pose_and_retry")
            return
        request = self.action_type.Goal()
        if self.allow_motion:
            request.pose = message
        else:
            request.goal, request.planner_id, request.use_start = message,"GridBased",False
        future = self.client.send_goal_async(request)
        future.add_done_callback(lambda f:self.accepted(f,generation))
        self.report("goal_submitted",x=p.x,y=p.y)

    def accepted(self, future, generation):
        try:
            handle = future.result()
        except Exception as exc:
            if generation == self.generation:
                self.report("goal_transport_error", detail=str(exc)[:160])
            return
        if generation != self.generation:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.report("goal_rejected")
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f:self.finished(f,generation))

    def finished(self, future, generation):
        if generation != self.generation:
            return
        self.goal_handle = None
        try:
            result = future.result()
        except Exception as exc:
            self.report("goal_result_error", detail=str(exc)[:160])
            return
        if result.status != 4:
            code = getattr(result.result,"error_code",None)
            names = ("UNKNOWN","INVALID_PLANNER","TF_ERROR","START_OUTSIDE_MAP","GOAL_OUTSIDE_MAP",
                     "START_OCCUPIED","GOAL_OCCUPIED","NO_VALID_PATH","TIMEOUT")
            error_name = next((name for name in names if getattr(type(result.result),name,None)==code),"UNKNOWN")
            self.report("goal_failed",action_status=result.status,error_code=code,error_name=error_name,
                        error_msg=str(getattr(result.result,"error_msg",""))[:240])
            return
        if not self.allow_motion:
            self.path_pub.publish(result.result.path)
            self.report("path_ready_preview_only",path_points=len(result.result.path.poses))
        else:
            self.report("goal_reached")


def main(args=None):
    rclpy.init(args=args)
    node = GoalDispatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
        node.destroy_node()
        rclpy.try_shutdown()
