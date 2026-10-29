"""Read current segmentation/goal/footprint; never publish or change parameters."""
import argparse
import ast
import json
import math
from pathlib import Path
import sys
import time
import uuid

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

import cv2
import numpy as np

from husky_drivable_area.ground_geometry import CameraCalibration,CameraMount,ground_to_image
from husky_drivable_area.robot_profile import expanded_path
from husky_drivable_area.vision_grid import decode_segmentation
from husky_drivable_area.control import quaternion_matrix


def ground_labels(points,labels,calibration,mount):
    raw,visible=ground_to_image(points,calibration,mount)
    answer=np.full(len(points),255,np.uint8)
    indices=np.flatnonzero(visible)
    if len(indices):
        rays=(np.column_stack([points[indices],np.zeros(len(indices))])-mount.origin)@mount.rotation
        expected=rays[:,:2]/rays[:,2,None]
        inverse=cv2.undistortPoints(raw[indices].reshape(-1,1,2),calibration.matrix,calibration.distortion).reshape(-1,2)
        consistent=np.linalg.norm((expected-inverse)*[calibration.matrix[0,0],calibration.matrix[1,1]],axis=1)<=.5
        indices=indices[consistent]
        pixels=(raw[indices]+.5)*[labels.shape[1]/calibration.width,labels.shape[0]/calibration.height]-.5
        u,v=np.rint(pixels).astype(int).T
        inside=(u>=0)&(u<labels.shape[1])&(v>=0)&(v<labels.shape[0])
        answer[indices[inside]]=labels[v[inside],u[inside]]
    return answer


def summarize_sections(sample,half_width,side_margin,goal_xy=None):
    """Widths are appearance evidence on the assumed plane, not verified terrain."""
    ys=np.linspace(-2.,2.,161);step=ys[1]-ys[0]
    sections=[]
    for x in (.75,1.,1.5,2.,2.5):
        labels=sample(np.column_stack([np.full(len(ys),x),ys]))
        free=labels==1
        edges=np.flatnonzero(np.diff(np.r_[False,free,False].astype(int)))
        intervals=[(max(-2.,ys[a]-step/2),min(2.,ys[b-1]+step/2)) for a,b in zip(edges[::2],edges[1::2])]
        central=next(((lo,hi) for lo,hi in intervals if lo<=0<=hi),None)
        item=dict(forward_x_m=x,axis_drivable=bool(labels[80]==1),
            observed_samples=int((labels!=255).sum()),
            width_around_robot_axis_m=None if central is None else round(central[1]-central[0],3),
            right_edge_y_m=None if central is None else round(central[0],3),
            left_edge_y_m=None if central is None else round(central[1],3),
            corridor_center_y_m=None if central is None else round((central[0]+central[1])/2,3),
            reserved_width_fits_at_axis=bool(central is not None and central[0]<=-half_width-side_margin and central[1]>=half_width+side_margin))
        if goal_xy is not None and goal_xy[0]>.05 and x<=goal_xy[0]:
            y=x*goal_xy[1]/goal_xy[0]
            item["goal_line_y_m"]=round(float(y),3)
            sample_value=int(sample(np.array([[x,y]]))[0])
            item["goal_line_label"]={0:"veto",1:"drivable",255:"unobserved"}[sample_value]
            item["reserved_width_fits_at_goal_line"]=any(lo<=y-half_width-side_margin and hi>=y+half_width+side_margin for lo,hi in intervals)
        sections.append(item)
    return dict(reserved_width_m=round(2*(half_width+side_margin),3),
        goal_ground_xy_m=None if goal_xy is None else np.round(goal_xy,3).tolist(),
        goal_angle_from_robot_forward_deg=None if goal_xy is None else round(math.degrees(math.atan2(goal_xy[1],goal_xy[0])),2),
        sections=sections,diagnostic_only=True,ready_for_control=False,
        note="Widths inferred from current RGB labels and the configured flat-ground geometry. No footprint sweep, physical alignment or terrain certification.")


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile",required=True)
    parser.add_argument("--duration",type=float,default=6.)
    args=parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 2<=args.duration<=15:
        parser.error("duration must be 2..15 seconds")
    profile=json.loads(Path(args.profile).expanduser().read_text())
    calibration=CameraCalibration.load(expanded_path(profile["camera"]["calibration_path"]))
    mount=CameraMount.load(expanded_path(profile["camera"]["mount_path"]))
    hull=profile["footprint"];half=hull["half_width_m"];margin=hull.get("lateral_margin_m",hull["margin_m"])
    if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in (half,margin)):
        raise ValueError("positive measured half-width and side margin required")
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from tf2_ros import Buffer,TransformListener,TransformException
    rclpy.init(args=[])
    node=Node("corridor_check_"+uuid.uuid4().hex[:8])
    buffer=Buffer();listener=TransformListener(buffer,node)
    latest={}
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions=[node.create_subscription(String,"/drivable_area/segmentation_packet",lambda m:latest.update(packet=m),1),
        node.create_subscription(PoseStamped,"/husky_nav/goal",lambda m:latest.update(goal=m),qos)]
    client=node.create_client(GetParameters,"/global_costmap/global_costmap/get_parameters")
    future=None;report=None;last_error="waiting_for_current_segmentation"
    try:
        end=time.monotonic()+args.duration
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.03)
            if future is None and client.service_is_ready():
                future=client.call_async(GetParameters.Request(names=["footprint"]))
            if "packet" not in latest:
                continue
            try:
                packet,labels=decode_segmentation(latest["packet"].data)
                stamp=packet["source_header"]["stamp"]
                if (type(stamp["sec"]) is not int or type(stamp["nanosec"]) is not int or
                        stamp["sec"]<0 or not 0<=stamp["nanosec"]<10**9):
                    raise ValueError("invalid_segmentation_timestamp")
                ns=stamp["sec"]*10**9+stamp["nanosec"]
                age=(node.get_clock().now().nanoseconds-ns)/1e9
                if ns<=0 or not -.05<=age<=.5:
                    raise ValueError("current_segmentation_not_fresh")
                if (packet["source_header"]["frame_id"]!=mount.camera_frame or
                        tuple(packet["raw_shape_hw"])!=(calibration.height,calibration.width)):
                    raise ValueError("current_camera_differs_from_calibration")
                goal_xy=None;goal_error="waiting_for_goal"
                if "goal" in latest:
                    goal=latest["goal"]
                    try:
                        tf=buffer.lookup_transform(mount.ground_frame,goal.header.frame_id,
                            Time(nanoseconds=ns,clock_type=node.get_clock().clock_type))
                        q,t=tf.transform.rotation,tf.transform.translation
                        rotation=quaternion_matrix(q.x,q.y,q.z,q.w)
                        if abs(rotation[2,2]-1)>1e-3:
                            raise ValueError("nonplanar_goal_transform")
                        goal_xy=rotation[:2,:2]@np.array([goal.pose.position.x,goal.pose.position.y])+[t.x,t.y]
                        if not np.isfinite(goal_xy).all():
                            raise ValueError("nonfinite_goal_coordinates")
                        goal_error=None
                    except TransformException as exc:
                        goal_error=str(exc)[:160]
                report=summarize_sections(lambda points:ground_labels(points,labels,calibration,mount),half,margin,goal_xy)
                report.update(source_age_ms=round(age*1000,1),source_stamp_ns=ns,
                    goal_projection_error=goal_error,profile_side_margin_m=margin)
                if future is not None and future.done():
                    if future.exception() is not None:raise ValueError(str(future.exception()))
                    corners=np.asarray(ast.literal_eval(future.result().values[0].string_value),float)
                    if corners.ndim!=2 or corners.shape[1]!=2 or not np.isfinite(corners).all():
                        raise ValueError("invalid_runtime_footprint")
                    report["runtime_costmap_reserved_width_m"]=round(2*float(np.abs(corners[:,1]).max()),3)
                    report["runtime_width_matches_profile"]=math.isclose(report["runtime_costmap_reserved_width_m"],report["reserved_width_m"],abs_tol=1e-5)
                    if goal_error is None:
                        break
            except (ValueError,TypeError,KeyError,IndexError,AttributeError,SyntaxError,cv2.error) as exc:
                last_error=str(exc)[:180]
        if report is None:
            print(json.dumps(dict(error=last_error,diagnostic_only=True,ready_for_control=False),indent=2));return 2
        report["source_age_at_report_ms"]=round((node.get_clock().now().nanoseconds-report["source_stamp_ns"])/1e6,1)
        report["capture_still_fresh"]= -.05<=report["source_age_at_report_ms"]/1000<=.5
        print(json.dumps(report,indent=2,allow_nan=False));return 0
    finally:
        node.destroy_node();rclpy.try_shutdown()


if __name__=="__main__":
    raise SystemExit(main())
