"""Ideal image-label junction simulation. No ROS or robot command publishers."""
import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from husky_drivable_area.vision_goal import plan_vision_reference
from husky_drivable_area.vision_local import local_goal_target
from husky_drivable_area.control import ControllerConfig,pursuit


def run_case(sign):
    pose=np.zeros(3);goal=np.array([2.,sign*3.]);trajectory=[pose.copy()]
    config=ControllerConfig(cruise_speed_mps=.15,max_angular_speed_rps=.15,lookahead_m=.5,max_path_offset_m=1.5)
    rotation=0.;arc_frames=0
    for step in range(700):
        c,s=np.cos(pose[2]),np.sin(pose[2]);matrix=np.array([[c,-s],[s,c]])
        def sample(points):
            world=points@matrix.T+pose[:2]
            return (((np.abs(world[:,1])<.75)&(world[:,0]<2.75))|
                ((np.abs(world[:,0]-2)<.75)&(world[:,1]*sign>-.75))).astype(np.uint8)
        local=(goal-pose[:2])@matrix
        if np.linalg.norm(local)<.15:break
        path,reason,detail=plan_vision_reference(sample,.8,local_goal_target(local,allow_side_goal=True),goal_priority=True)
        if not len(path):raise AssertionError((step,pose.tolist(),reason))
        arc_frames+=detail.get('reference_kind')=='junction_arc'
        demand=pursuit(path,[0,0,0],config)
        if demand.reason!='tracking':raise AssertionError(demand.reason)
        pose[:2]+=.1*demand.linear*np.array([c,s]);pose[2]+=.1*demand.angular
        rotation+=abs(.1*demand.angular);trajectory.append(pose.copy())
    error=float(np.linalg.norm(goal-pose[:2]))
    assert error<.15 and sign*pose[2]>math.pi/3
    assert rotation<2*math.pi and arc_frames>0
    return dict(turn='left' if sign>0 else 'right',goal_xy_m=goal.tolist(),error_m=error,
        final_heading_deg=float(np.rad2deg(pose[2])),cumulative_rotation_rad=rotation,
        junction_arc_frames=arc_frames,steps=step,synthetic=True,physical_robot=False),np.array(trajectory)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output-dir',required=True)
    args=parser.parse_args();folder=Path(args.output_dir);folder.mkdir(parents=True,exist_ok=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(10,5));results=[]
    for sign,ax in zip((1,-1),axes):
        result,trace=run_case(sign);results.append(result)
        ax.fill_between([-1,2.75],-.75,.75,color='#d9f0dc')
        ax.fill_between([1.25,2.75],-.75 if sign>0 else -4,4 if sign>0 else .75,color='#d9f0dc')
        ax.plot(trace[:,0],trace[:,1],color='#cc9e00',label='robot trajectory')
        ax.scatter(*result['goal_xy_m'],c='red',marker='x',label='goal')
        ax.set(title=result['turn']+' junction',xlabel='world X (m)',ylabel='world Y (m)',aspect='equal')
        ax.set_xlim(-.5,3.5);ax.set_ylim(-4,4);ax.grid(alpha=.2);ax.legend()
    fig.suptitle('Synthetic image-label junctions; no physical robot, no ROS commands')
    fig.tight_layout();fig.savefig(folder/'junction-turns.png',dpi=160)
    (folder/'report.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results,indent=2))


if __name__=='__main__':main()
