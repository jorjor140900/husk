"""Explain an existing plan against the published map/costmap; read-only."""
import argparse
import json
import math
import time

import numpy as np


def _sample_grid(grid, points):
    info=grid.info
    p,q=info.origin.position,info.origin.orientation
    if (not 0<info.resolution<=1 or info.width<=0 or info.height<=0 or not 0<info.width*info.height<=2_000_000 or
            len(grid.data)!=info.width*info.height or
            not np.isfinite([p.x,p.y,q.x,q.y,q.z,q.w]).all() or
            abs(q.x)>1e-6 or abs(q.y)>1e-6 or abs(q.z*q.z+q.w*q.w-1)>1e-5):
        raise ValueError("invalid planar occupancy grid")
    yaw=2*math.atan2(q.z,q.w);c,s=math.cos(yaw),math.sin(yaw)
    local=(points-[p.x,p.y])@np.array([[c,-s],[s,c]])
    cells=np.floor(local/info.resolution).astype(int)
    inside=(cells[:,0]>=0)&(cells[:,0]<info.width)&(cells[:,1]>=0)&(cells[:,1]<info.height)
    costs=np.full(len(points),-1,dtype=int)
    data=np.asarray(grid.data)
    if ((data < -1)|(data > 100)).any():
        raise ValueError("grid values must be -1..100")
    costs[inside]=data[cells[inside,1]*info.width+cells[inside,0]]
    return costs,inside,cells


def analyze_plan(path,costmap,saved_map):
    if not 2<=len(path.poses)<=4096 or not path.header.frame_id:
        raise ValueError("need a nonempty current goal plan")
    if path.header.frame_id!=costmap.header.frame_id or path.header.frame_id!=saved_map.header.frame_id:
        raise ValueError("path/map/costmap frames must match")
    if any(p.header.frame_id not in ("",path.header.frame_id) for p in path.poses):
        raise ValueError("plan pose frames must match path frame")
    points=np.array([[p.pose.position.x,p.pose.position.y] for p in path.poses],float)
    if not np.isfinite(points).all():
        raise ValueError("plan contains nonfinite positions")
    distance=float(np.linalg.norm(points[-1]-points[0]))
    length=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum())
    if not math.isfinite(distance) or not math.isfinite(length):
        raise ValueError("plan distances must be finite")
    if distance<.01:
        raise ValueError("goal is already at start; no meaningful straight-line comparison")
    step=min(costmap.info.resolution,saved_map.info.resolution)/2
    if not math.isfinite(step) or step<.005 or distance/step>20000:
        raise ValueError("unsupported comparison sampling size")
    ray=np.linspace(points[0],points[-1],math.ceil(distance/step)+1)
    costs,inside,cells=_sample_grid(costmap,ray)
    # Count each costmap cell once; ray samples are diagnostic, not a swept
    # footprint collision certificate. Never change a path or cost to pass.
    _,indices=np.unique(cells,axis=0,return_index=True)
    indices=np.sort(indices);ray,costs,inside=ray[indices],costs[indices],inside[indices]
    mapped,map_inside,_=_sample_grid(saved_map,ray)
    blocked=inside&(costs>=99)
    soft=inside&(costs>0)&(costs<99)
    offset=points-points[0]
    vector=(points[-1]-points[0])/distance
    lateral=np.abs(offset[:,0]*vector[1]-offset[:,1]*vector[0])
    return dict(frame_id=path.header.frame_id,path_points=len(points),
        path_start_xy_m=points[0].tolist(),path_end_xy_m=points[-1].tolist(),
        direct_distance_m=round(distance,3),plan_length_m=round(length,3),
        detour_ratio=round(length/distance,3),max_lateral_departure_m=round(float(lateral.max()),3),
        straight_line=dict(sampled_cells=len(ray),outside_costmap_cells=int((~inside).sum()),
            unknown_cells=int((inside&(costs<0)).sum()),blocked_cells=int(blocked.sum()),
            soft_cost_cells=int(soft.sum()),max_known_cost=int(costs[inside].max()) if inside.any() else None,
            saved_map_occupied_cells=int((map_inside&(mapped>=99)).sum()),
            blocked_but_free_in_saved_map=int((blocked&map_inside&(mapped==0)).sum()),
            blocked_xy_examples_m=np.round(ray[blocked][:8],3).tolist()),
        diagnostic_only=True,ready_for_control=False,
        note="Point-sampled direct line through the existing published grids; no footprint/terrain/obstacle-safety certification.")


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration",type=float,default=4.)
    args=parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 2<=args.duration<=15:
        parser.error("duration must be 2..15 seconds")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from nav_msgs.msg import Path,OccupancyGrid
    rclpy.init()
    node=Node("navigation_plan_check")
    latest={}
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions=[node.create_subscription(cls,topic,lambda msg,key=key:latest.update({key:msg}),qos)
        for key,cls,topic in (("path",Path,"/husky_nav/goal_plan"),
            ("costmap",OccupancyGrid,"/global_costmap/costmap"),("map",OccupancyGrid,"/map"))]
    try:
        end=time.monotonic()+args.duration
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
        missing=set(("path","costmap","map"))-latest.keys()
        if missing:
            raise ValueError("missing topics: "+", ".join(sorted(missing)))
        report=analyze_plan(latest["path"],latest["costmap"],latest["map"])
        now=node.get_clock().now().nanoseconds
        report["source_age_ms"]={key:round((now-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e6,1)
                                 for key,msg in latest.items()}
        print(json.dumps(report,indent=2,allow_nan=False))
        return 0
    except (ValueError,TypeError,AttributeError,KeyError) as exc:
        print(json.dumps(dict(error=str(exc),diagnostic_only=True,ready_for_control=False)))
        return 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
