"""Exercise local image/LiDAR planning without ROS, a camera, a robot or a map.

All image labels and sensor rays here are synthetic. This is a software demo,
not a recorded field result or a motion-permission tool.
"""
import argparse
from datetime import datetime,timezone
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from husky_drivable_area.collision import circle_scan
from husky_drivable_area.control import ControllerConfig,CommandLimiter,Demand,pursuit
from husky_drivable_area.vision_goal import plan_vision_reference
from husky_drivable_area.vision_local import LocalLidarGrid,fused_sampler,local_goal_target


def room_scan(pose=(0.,0.,0.),circles=(),count=900):
    """Actual intersections with fixed synthetic walls at world X/Y = +/-10 m."""
    x,y,yaw=pose
    angles=-math.pi+np.arange(count)*2*math.pi/count+yaw
    directions=np.column_stack([np.cos(angles),np.sin(angles)])
    sides=np.where(directions>=0,10.,-10.)
    with np.errstate(divide='ignore',invalid='ignore'):
        distances=(sides-[x,y])/directions
    walls=np.min(np.where(distances>0,distances,np.inf),axis=1)
    ranges=np.minimum(walls,circle_scan(pose,circles,count=count,range_max=50.))
    return NS(ranges=ranges,angle_min=-math.pi,angle_increment=2*math.pi/count,
        angle_max=-math.pi+(count-1)*2*math.pi/count,range_min=.1,range_max=50.,
        scan_time=0.,time_increment=0.)


def corridor(points):
    return np.where(np.abs(points[:,1])<1.2,1,0).astype(np.uint8)


def plan(grid,image=corridor,goal=(4.,0.)):
    return plan_vision_reference(fused_sampler(image,grid,np.eye(4)),1.,local_goal_target(goal),
        candidate_check=lambda path:grid.reference_clear(path,np.eye(4),1.))


def simulate():
    pose=np.zeros(3);goal=np.array([2.,0.]);states=[pose.copy()]
    config=ControllerConfig(cruise_speed_mps=.15,max_angular_speed_rps=.2,max_path_offset_m=1.5,lookahead_m=.5)
    limiter=CommandLimiter(config);measured=(0.,0.);dt=.1
    for _ in range(220):
        distance=float(np.linalg.norm(goal-pose[:2]))
        if distance<=.15:break
        c,s=math.cos(pose[2]),math.sin(pose[2]);rotation=np.array([[c,-s],[s,c]])
        grid=LocalLidarGrid(room_scan(pose),np.eye(4))
        def image(points):return corridor(points@rotation.T+pose[:2])
        path,reason,_=plan(grid,image,(goal-pose[:2])@rotation)
        if not len(path):raise RuntimeError('synthetic planning stopped: '+reason)
        reference=path@rotation.T+pose[:2]
        demand=pursuit(reference,pose,config)
        speed=min(demand.linear,math.sqrt(2*config.max_deceleration_mps2*max(distance-.15,0)))
        angular=demand.angular*speed/max(demand.linear,1e-9)
        demand=Demand(speed,angular,demand.reason)
        v,w=limiter.step(demand,dt)
        if not grid.command_clear((v,w),measured,np.eye(4),0.,(.15,.2)):
            raise RuntimeError('synthetic command blocked by local scan-plane sweep')
        pose[:2]+=[v*math.cos(pose[2])*dt,v*math.sin(pose[2])*dt];pose[2]+=w*dt
        measured=(v,w);states.append(pose.copy())
    error=float(np.linalg.norm(goal-pose[:2]))
    if error>.15:raise RuntimeError('synthetic goal not reached')
    return states,error


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path)
    args=parser.parse_args(argv)
    output=args.output_dir or Path('outputs')/('local-fusion-demo-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    output.mkdir(parents=True,exist_ok=False)
    (output/'COLCON_IGNORE').touch()
    clear=LocalLidarGrid(room_scan(),np.eye(4));straight,_,_=plan(clear)
    far=LocalLidarGrid(room_scan(circles=[(3.3,0.,.15)]),np.eye(4));detour,_,_=plan(far)
    near=LocalLidarGrid(room_scan(circles=[(.85,0.,.15)]),np.eye(4))
    missing=room_scan();missing.ranges[:]=np.inf;unknown=LocalLidarGrid(missing,np.eye(4))
    unknown_path,_,_=plan(unknown)
    veto_path,_,_=plan(clear,lambda p:np.zeros(len(p),np.uint8))
    assert len(straight)>20 and np.max(np.abs(straight[:,1]))<.1
    assert len(detour)>20 and abs(detour[-1,1])>.4
    assert not near.command_clear((.15,0),(0,0),np.eye(4),.1,(.15,.2))
    assert not len(unknown_path) and not len(veto_path)
    states,error=simulate()
    report=dict(result='passed',synthetic=True,physical_robot=False,global_plan_required=False,
        straight_corridor=True,far_obstacle_changes_reference=True,near_obstacle_stops=True,
        missing_rays_remain_unknown=True,image_veto_stops=True,synthetic_goal_error_m=error,
        synthetic_goal_reached=True,synthetic_goal_distance_m=2.,
        actual_commands_published=False,ready_for_autonomous_control=False,
        note='Ideal instantaneous scan-plane rays and synthetic floor labels. No terrain/height/physical stopping validation.')
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    np.savetxt(output/'synthetic-trajectory.csv',np.array(states),delimiter=',',header='x_m,y_m,yaw_rad')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(13,4.5),layout='constrained')
    for ax,path,grid,title in zip(axes,[straight,detour,unknown_path],[clear,far,unknown],
            ['Clear lane: direct goal reference','Live obstacle: reference bends','Missing rays: STOP / unknown']):
        ax.axhspan(-1.2,1.2,color='#d7efd9');ax.scatter(grid.points[:,0],grid.points[:,1],s=6,color='#a52da5')
        if len(path):ax.plot(path[:,0],path[:,1],color='#d3a000',linewidth=3,label='Controller reference')
        ax.scatter([0],[0],marker='+',s=80,color='#1a53bf');ax.scatter([4],[0],marker='x',color='#d43f3f')
        ax.set(xlim=(-.6,4.2),ylim=(-1.5,1.5),xlabel='Forward X (m)',ylabel='Left Y (m)',title=title)
        ax.set_aspect('equal');ax.grid(alpha=.2)
    fig.suptitle('Synthetic local image + measured-ray fusion — no global path, no robot',fontsize=13)
    fig.savefig(output/'local-fusion-demo.png',dpi=170);plt.close(fig)
    print(json.dumps(dict(**report,output_dir=str(output)),indent=2))
    return 0


if __name__=='__main__':raise SystemExit(main())
