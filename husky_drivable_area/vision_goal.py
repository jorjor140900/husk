"""Metric image-first references. Map routes select direction, not steering.

Only observed image labels can support a reference; unknown pixels and vetoes
are never filled. These lateral-clearance checks do not replace the complete
robot footprint/braking checks in the downstream collision filter.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np

from .control import validate_path
from .ground_geometry import ground_to_image


def validate_guidance_mode(mode,vision=True,supervised_trial=False):
    if mode not in ("global_route","local_fusion","image_only"):
        raise ValueError("guidance_mode must be global_route, local_fusion or image_only")
    if mode in ("local_fusion","image_only") and not vision:
        raise ValueError(mode+" requires vision steering")
    if mode=="image_only" and supervised_trial is not True:
        raise ValueError("image_only requires explicit supervised_trial")


@dataclass(frozen=True)
class VisionReferenceConfig:
    half_width_m: float = .33
    side_margin_m: float = .08
    max_forward_m: float = 2.8
    max_lateral_m: float = 1.2
    step_m: float = .025
    min_length_m: float = .5
    max_curvature: float = 1.5

    def __post_init__(self):
        if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0
               for v in self.__dict__.values()):
            raise ValueError("vision reference dimensions must be finite and positive")
        if not .01<=self.step_m<=.05 or not self.min_length_m<self.max_forward_m<=4:
            raise ValueError("invalid vision reference sampling limits")
        if self.max_lateral_m>2 or self.half_width_m+self.side_margin_m>=self.max_lateral_m:
            raise ValueError("invalid vision reference width")


def sample_ground_labels(points, labels, calibration, mount):
    """Distortion-consistent projection, including explicit unseen label 255."""
    points=np.asarray(points,float).reshape(-1,2)
    sampled=np.full(len(points),255,np.uint8)
    for start in range(0,len(points),10000):
        ground=points[start:start+10000]
        raw,visible=ground_to_image(ground,calibration,mount)
        indices=np.flatnonzero(visible)
        if not len(indices):
            continue
        camera=(np.column_stack([ground[indices],np.zeros(len(indices))])-mount.origin)@mount.rotation
        expected=camera[:,:2]/camera[:,2,None]
        recovered=cv2.undistortPoints(raw[indices].reshape(-1,1,2),calibration.matrix,
                                    calibration.distortion).reshape(-1,2)
        consistent=np.linalg.norm((expected-recovered)*[calibration.matrix[0,0],calibration.matrix[1,1]],axis=1)<=.5
        indices=indices[consistent]
        pixels=(raw[indices]+.5)*[labels.shape[1]/calibration.width,labels.shape[0]/calibration.height]-.5
        u,v=np.rint(pixels).astype(int).T
        inside=(u>=0)&(u<labels.shape[1])&(v>=0)&(v<labels.shape[0])
        sampled[start+indices[inside]]=labels[v[inside],u[inside]]
    return sampled


def route_direction(route_ground, goal_ground, lookahead=2.5):
    """A coarse waypoint ahead of the nearest route projection, never behind."""
    goal=np.asarray(goal_ground,float)
    if goal.shape!=(2,) or not np.isfinite(goal).all():
        raise ValueError("invalid_goal")
    if goal[0]<=0:
        raise ValueError("goal_not_ahead")
    path=validate_path(route_ground,.5)
    segments=np.diff(path,axis=0)
    fractions=np.clip(-np.sum(path[:-1]*segments,axis=1)/np.sum(segments**2,axis=1),0,1)
    nearest=path[:-1]+fractions[:,None]*segments
    index=int(np.argmin(np.linalg.norm(nearest,axis=1)))
    ahead=np.vstack([nearest[index],path[index+1:]])
    arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(ahead,axis=0),axis=1))]
    distance=min(lookahead,arc[-1])
    target=np.array([np.interp(distance,arc,ahead[:,i]) for i in (0,1)])
    if np.linalg.norm(goal)<lookahead:
        target=goal
    if target[0]<=.05 or abs(math.atan2(target[1],target[0]))>math.radians(65):
        raise ValueError("route_requires_reorientation")
    return target


def plan_vision_reference(sample, start_x, target, config=VisionReferenceConfig(),candidate_check=None,*,goal_priority=False):
    """Smooth bounded candidates scored by corridor clearance and goal direction.

    The reference starts on the measured robot axis at the calibrated near
    image row, not at a relocated free blob. No connector over the camera's
    unseen near field is invented. The controller/collision filter must handle
    that explicitly, and stop when no continuous observed reference exists.
    """
    empty=np.empty((0,2))
    if not math.isfinite(start_x) or not .05<=start_x<config.max_forward_m-config.min_length_m:
        return empty,"near_axis_not_observable",{}
    target=np.asarray(target,float)
    if target.shape!=(2,) or not np.isfinite(target).all() or (target[0]<=0 and not goal_priority):
        return empty,"target_not_ahead",{}
    half=config.half_width_m+config.side_margin_m
    offsets=np.linspace(-half,half,int(math.ceil(2*half/config.step_m))+1)
    seed=np.column_stack([np.full(len(offsets),start_x),offsets])
    if not np.all(sample(seed)==1):
        return empty,"start_blocked_narrow_or_unobserved",{}
    # A compact bird's-eye sampling grid supplies a corridor-centering cost.
    forward=config.max_forward_m
    if goal_priority and target[0]>=start_x+config.min_length_m:
        forward=min(forward,float(target[0]))
    xs=np.arange(start_x,forward+config.step_m/2,config.step_m)
    if goal_priority:xs=xs[xs<=forward+1e-9]
    ys=np.arange(-config.max_lateral_m,config.max_lateral_m+config.step_m/2,config.step_m)
    grid_xs=np.arange(0. if goal_priority else start_x,config.max_forward_m+config.step_m/2,config.step_m)
    xx,yy=np.meshgrid(grid_xs,ys,indexing="ij")
    labels=sample(np.column_stack([xx.ravel(),yy.ravel()])).reshape(xx.shape)
    # Image-only supervision supplies the unseen near connector for a turn.
    # Keep known image vetoes; never extend this assumption beyond start_x.
    observed=(labels==1)|((labels==255)&(xx<start_x)&goal_priority)
    clearance=cv2.distanceTransform(np.pad(observed.astype(np.uint8),1),cv2.DIST_L2,3)[1:-1,1:-1]*config.step_m
    def strip_observed(points):
        # Cache calibrated projection once. Require all four neighbouring grid
        # samples, rather than interpolating a veto/unknown into free space.
        coords=(points-[grid_xs[0],ys[0]])/config.step_m
        if ((coords < -1e-8).any() or
                (coords > np.array(observed.shape)-1+1e-8).any()):
            return False
        coords=np.clip(coords,[0,0],np.array(observed.shape)-1)
        low=np.floor(coords).astype(int);high=np.ceil(coords).astype(int)
        return all(observed[ix,iy].all() for ix,iy in (
            (low[:,0],low[:,1]),(low[:,0],high[:,1]),
            (high[:,0],low[:,1]),(high[:,0],high[:,1])))
    slope=target[1]/max(target[0],.01)
    ends=np.unique(np.r_[np.arange(-config.max_lateral_m+half,config.max_lateral_m-half,.05),
                             0.,np.clip(slope*xs[-1],-config.max_lateral_m+half,config.max_lateral_m-half)])
    best=None
    # Prefer the longest observed prefix. A short reference never jumps a veto
    # to reach a visually disconnected branch farther away.
    for end_index in range(len(xs)-1,1,-4):
        end=xs[end_index]
        if end-start_x<config.min_length_m:
            break
        # No candidate at this or a shorter X can improve the goal distance
        # already achieved. Avoid evaluating every short prefix each frame.
        if goal_priority and best is not None and abs(target[0]-end)>best[0][0]+1e-9:
            break
        x=xs[:end_index+1]
        t=(x-start_x)/(end-start_x)
        blend=3*t*t-2*t*t*t
        candidates=ends
        if goal_priority:
            candidates=np.unique(np.r_[ends,np.clip(slope*end,
                -config.max_lateral_m+half,config.max_lateral_m-half)])
        for y_end in candidates:
            y=y_end*blend
            dy=y_end*6*t*(1-t)/(end-start_x)
            ddy=y_end*(6-12*t)/(end-start_x)**2
            curvature=ddy/(1+dy*dy)**1.5
            if np.max(np.abs(curvature))>config.max_curvature:
                continue
            normals=np.column_stack([-dy,np.ones(len(x))])/np.sqrt(1+dy*dy)[:,None]
            points=np.column_stack([x,y])
            goal_error=float(np.linalg.norm(points[-1]-target))
            if goal_priority and best is not None and goal_error>best[0][0]+1e-12:
                continue
            strip=points[:,None,:]+normals[:,None,:]*offsets[None,:,None]
            if not strip_observed(strip.reshape(-1,2)):
                continue
            if candidate_check is not None and not candidate_check(points):
                continue
            yi=np.clip(np.rint((y-ys[0])/config.step_m).astype(int),0,len(ys)-1)
            xi=np.rint((x-grid_xs[0])/config.step_m).astype(int)
            distance=clearance[xi,yi]
            centre=float(np.mean(1-np.minimum(distance/.6,1)))
            angle=math.atan2(y_end,end)
            goal_angle=math.atan2(target[1],target[0])
            # Corridor centering dominates small guidance deviations; route
            # direction chooses branches, not a metric global path to copy.
            score=centre+.65*abs(angle-goal_angle)+.03*float(np.mean(curvature**2))
            goal_error=float(np.linalg.norm(points[-1]-target))
            if goal_priority:
                # First approach the clicked goal; centering/smoothness only
                # resolve equally good endpoints. All mask/curve checks above
                # remain mandatory, including for a straight-to-goal candidate.
                score=(goal_error,centre+.03*float(np.mean(curvature**2)))
            if best is None or score<best[0]:
                best=(score,points,dict(reference_start_m=float(start_x),
                    observed_reference_length_m=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum()),
                    reference_forward_span_m=float(end-start_x),reference_max_lateral_m=float(np.abs(y).max()),
                    reserved_width_m=float(2*half),
                    max_curvature=float(np.max(np.abs(curvature))),
                    guide_target_ground_xy=target.tolist(),footprint_checked=False,
                    goal_priority=goal_priority,reference_end_ground_xy=points[-1].tolist(),
                    reference_end_to_target_m=goal_error,
                    unseen_near_field_m=float(start_x)))
        if best is not None and not goal_priority:
            return best[1],"ok",best[2]
    if goal_priority:
        # Turning candidates keep a nonzero terminal heading. Unlike the
        # lane-change smoothstep above they can follow a visible junction arm.
        for sign in (-1,1):
            for radius in np.arange(max(1/config.max_curvature,start_x+.05),2.01,.15):
                for degrees in (25,40,55,70,85):
                    angle=math.radians(degrees);begin=math.asin(start_x/radius)
                    length=radius*(angle-begin)
                    if length<config.min_length_m:continue
                    a=np.linspace(begin,angle,int(math.ceil(length/config.step_m))+1)
                    points=np.column_stack([radius*np.sin(a),sign*radius*(1-np.cos(a))])
                    goal_error=float(np.linalg.norm(points[-1]-target))
                    if best is not None and goal_error>best[0][0]+1e-12:continue
                    normals=np.column_stack([-sign*np.sin(a),np.cos(a)])
                    if not strip_observed((points[:,None,:]+normals[:,None,:]*offsets[None,:,None]).reshape(-1,2)):
                        continue
                    if candidate_check is not None and not candidate_check(points):continue
                    goal_error=float(np.linalg.norm(points[-1]-target))
                    score=(goal_error,.03/radius**2)
                    if best is None or score<best[0]:
                        best=(score,points,dict(reference_start_m=float(start_x),
                            observed_reference_length_m=float(length),reference_forward_span_m=float(points[-1,0]-start_x),
                            reference_max_lateral_m=float(np.abs(points[:,1]).max()),reserved_width_m=float(2*half),
                            max_curvature=float(1/radius),guide_target_ground_xy=target.tolist(),footprint_checked=False,
                            goal_priority=True,reference_end_ground_xy=points[-1].tolist(),reference_end_to_target_m=goal_error,
                            unseen_near_field_m=float(start_x),reference_kind="junction_arc",terminal_heading_rad=sign*angle))
    if best is not None:return best[1],"ok",best[2]
    return empty,"no_observed_image_corridor",{}
