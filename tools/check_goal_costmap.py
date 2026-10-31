"""Read-only current-goal diagnosis, including when Nav2 returned no path.

No velocity/permission publishers, no actions, no parameter writes.
Run directly from a sourced checkout; no colcon rebuild required.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import cv2
from husky_drivable_area.navigation_plan_check import _sample_grid


def analyze_goal(start,goal,costmap,saved):
    points=np.asarray([start,goal],float)
    if points.shape!=(2,2) or not np.isfinite(points).all():raise ValueError('invalid goal coordinates')
    if costmap.header.frame_id!='map' or saved.header.frame_id!='map':raise ValueError('grids must be in map')
    values,inside,cells=_sample_grid(costmap,points)
    mapped,map_inside,_=_sample_grid(saved,points)
    endpoints={}
    for i,name in enumerate(('start','goal')):
        cost=int(values[i])
        label='outside' if not inside[i] else ('unknown' if cost<0 else ('blocked' if cost>=99 else ('soft_cost' if cost>0 else 'free')))
        endpoints[name]=dict(xy_m=points[i].tolist(),costmap_label=label,costmap_cost=cost if inside[i] else None,
            saved_map_cost=int(mapped[i]) if map_inside[i] else None)
    distance=float(np.linalg.norm(points[1]-points[0]))
    step=min(costmap.info.resolution,saved.info.resolution)/2
    if step<.005 or distance/step>20000:raise ValueError('unsupported direct-line sampling size')
    ray=np.linspace(points[0],points[1],max(2,math.ceil(distance/step)+1))
    costs,on_map,indices=_sample_grid(costmap,ray)
    _,unique=np.unique(indices,axis=0,return_index=True)
    unique=np.sort(unique);ray,costs,on_map=ray[unique],costs[unique],on_map[unique]
    saved_costs,saved_inside,_=_sample_grid(saved,ray)
    blocked=on_map&(costs>=99)
    connected=None;reachable_cells=0
    if all(inside) and all(0<=v<99 for v in values):
        data=np.asarray(costmap.data).reshape(costmap.info.height,costmap.info.width)
        free=((data>=0)&(data<99)).astype(np.uint8)
        reachable_cells,filled,_,_=cv2.floodFill(free,None,tuple(int(x) for x in cells[0]),2,flags=4)
        connected=bool(filled[cells[1,1],cells[1,0]]==2)
    return dict(read_only=True,diagnostic_only=True,ready_for_control=False,
        frame_id='map',direct_distance_m=round(distance,3),**endpoints,
        straight_line=dict(sampled_cells=len(ray),blocked_cells=int(blocked.sum()),
            unknown_cells=int((on_map&(costs<0)).sum()),outside_costmap_cells=int((~on_map).sum()),
            saved_map_occupied_cells=int((saved_inside&(saved_costs>=99)).sum()),
            blocked_but_free_in_saved_map=int((blocked&saved_inside&(saved_costs==0)).sum()),
            blocked_xy_examples_m=np.round(ray[blocked][:8],3).tolist()),
        point_grid_connected_4_neighbour=connected,reachable_free_cells=int(reachable_cells),
        note='Point-grid evidence only; unknown excluded, cost>=99 blocked. No path is synthesized and no footprint sweep or safety certification.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=10.)
    parser.add_argument('--base-frame',default='base_link')
    parser.add_argument('--output-dir',default=str(Path.home()/'husky_vision_ws/outputs/goal-costmap'))
    args=parser.parse_args()
    if not math.isfinite(args.duration) or not 3<=args.duration<=20:parser.error('duration must be 3..20 s')
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    from tf2_ros import Buffer,TransformListener,TransformException
    from rcl_interfaces.srv import GetParameters
    rclpy.init();n=Node('read_only_goal_costmap_check');latest={};subscriptions=[]
    buffer=Buffer();listener=TransformListener(buffer,n)
    topics=dict(map='/map',costmap='/global_costmap/costmap',goal='/husky_nav/goal')
    for key,cls in (('map',OccupancyGrid),('costmap',OccupancyGrid),('goal',PoseStamped)):
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE if key=='costmap' else DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions.append(n.create_subscription(cls,topics[key],lambda m,k=key:latest.update({k:m}),qos))
    subscriptions.append(n.create_subscription(String,'/husky_nav/goal_status',lambda m:latest.update(status=m),1))
    client=n.create_client(GetParameters,'/global_costmap/global_costmap/get_parameters');future=None
    print('READ ONLY: collecting goal/map/costmap/TF; keep operator STOP. No existing plan needed.',flush=True)
    report={}
    try:
        end=time.monotonic()+args.duration
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.03)
            if future is None and client.service_is_ready():
                future=client.call_async(GetParameters.Request(names=['inflation_layer.inflation_radius','footprint']))
        missing=set(topics)-latest.keys()
        if missing:raise ValueError('missing topics: '+', '.join(sorted(missing)))
        goal=latest['goal']
        if goal.header.frame_id!='map':raise ValueError('goal must be in map')
        tf=buffer.lookup_transform('map',args.base_frame,Time())
        now=n.get_clock().now().nanoseconds
        ns=tf.header.stamp.sec*10**9+tf.header.stamp.nanosec
        age=(now-ns)/1e9
        if ns<=0 or not -.05<=age<=.3:raise ValueError('robot TF stale; source_age_ms=%.1f'%(age*1000))
        report=analyze_goal([tf.transform.translation.x,tf.transform.translation.y],
            [goal.pose.position.x,goal.pose.position.y],latest['costmap'],latest['map'])
        report['tf_age_ms']=round(age*1000,1)
        report['source_age_ms']={k:round((now-(m.header.stamp.sec*10**9+m.header.stamp.nanosec))/1e6,1)
            for k,m in latest.items() if hasattr(m,'header')}
        if future is not None and future.done() and future.exception() is None:
            values=future.result().values
            report['inflation_radius_m']=values[0].double_value if values[0].type==3 else None
            report['runtime_footprint']=values[1].string_value if values[1].type==4 else None
        if 'status' in latest:
            try:report['goal_status']=json.loads(latest['status'].data)
            except ValueError:pass
    except (ValueError,TypeError,AttributeError,TransformException) as e:
        report=dict(error=str(e),read_only=True,diagnostic_only=True,ready_for_control=False)
        report['publishers']={key:[dict(node=x.node_name,reliability=str(x.qos_profile.reliability),
            durability=str(x.qos_profile.durability)) for x in n.get_publishers_info_by_topic(topic)] for key,topic in topics.items()}
    finally:
        folder=Path(args.output_dir).expanduser();folder.mkdir(parents=True,exist_ok=True)
        filename=folder/('goal-costmap-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.json')
        filename.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print(json.dumps(report,indent=2,allow_nan=False));print('REPORT: '+str(filename))
        n.destroy_node();rclpy.try_shutdown()
    return 2 if 'error' in report else 0


if __name__=='__main__':raise SystemExit(main())
