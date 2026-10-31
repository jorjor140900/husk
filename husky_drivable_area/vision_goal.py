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


def plan_vision_reference(sample, start_x, target, config=VisionReferenceConfig()):
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
    if target.shape!=(2,) or not np.isfinite(target).all() or target[0]<=0:
        return empty,"target_not_ahead",{}
    half=config.half_width_m+config.side_margin_m
    offsets=np.linspace(-half,half,int(math.ceil(2*half/config.step_m))+1)
    seed=np.column_stack([np.full(len(offsets),start_x),offsets])
    if not np.all(sample(seed)==1):
        return empty,"start_blocked_narrow_or_unobserved",{}
    # A compact bird's-eye sampling grid supplies a corridor-centering cost.
    xs=np.arange(start_x,config.max_forward_m+config.step_m/2,config.step_m)
    ys=np.arange(-config.max_lateral_m,config.max_lateral_m+config.step_m/2,config.step_m)
    xx,yy=np.meshgrid(xs,ys,indexing="ij")
    observed=sample(np.column_stack([xx.ravel(),yy.ravel()])).reshape(xx.shape)==1
    clearance=cv2.distanceTransform(np.pad(observed.astype(np.uint8),1),cv2.DIST_L2,3)[1:-1,1:-1]*config.step_m
    def strip_observed(points):
        # Cache calibrated projection once. Require all four neighbouring grid
        # samples, rather than interpolating a veto/unknown into free space.
        coords=(points-[xs[0],ys[0]])/config.step_m
        if ((coords < -1e-8).any() or
                (coords > np.array(observed.shape)-1+1e-8).any()):
            return False
        coords=np.clip(coords,[0,0],np.array(observed.shape)-1)
        low=np.floor(coords).astype(int);high=np.ceil(coords).astype(int)
        return all(observed[ix,iy].all() for ix,iy in (
            (low[:,0],low[:,1]),(low[:,0],high[:,1]),
            (high[:,0],low[:,1]),(high[:,0],high[:,1])))
    ends=np.unique(np.r_[np.arange(-config.max_lateral_m+half,config.max_lateral_m-half,.05),
                             0.,np.clip(target[1]/target[0]*xs[-1],-config.max_lateral_m+half,config.max_lateral_m-half)])
    best=None
    # Prefer the longest observed prefix. A short reference never jumps a veto
    # to reach a visually disconnected branch farther away.
    for end_index in range(len(xs)-1,1,-4):
        end=xs[end_index]
        if end-start_x<config.min_length_m:
            break
        x=xs[:end_index+1]
        t=(x-start_x)/(end-start_x)
        blend=3*t*t-2*t*t*t
        for y_end in ends:
            y=y_end*blend
            dy=y_end*6*t*(1-t)/(end-start_x)
            ddy=y_end*(6-12*t)/(end-start_x)**2
            curvature=ddy/(1+dy*dy)**1.5
            if np.max(np.abs(curvature))>config.max_curvature:
                continue
            normals=np.column_stack([-dy,np.ones(len(x))])/np.sqrt(1+dy*dy)[:,None]
            points=np.column_stack([x,y])
            strip=points[:,None,:]+normals[:,None,:]*offsets[None,:,None]
            if not strip_observed(strip.reshape(-1,2)):
                continue
            yi=np.clip(np.rint((y-ys[0])/config.step_m).astype(int),0,len(ys)-1)
            distance=clearance[np.arange(len(x)),yi]
            centre=float(np.mean(1-np.minimum(distance/.6,1)))
            angle=math.atan2(y_end,end)
            goal_angle=math.atan2(target[1],target[0])
            # Corridor centering dominates small guidance deviations; route
            # direction chooses branches, not a metric global path to copy.
            score=centre+.65*abs(angle-goal_angle)+.03*float(np.mean(curvature**2))
            if best is None or score<best[0]:
                best=(score,points,dict(reference_start_m=float(start_x),
                    observed_reference_length_m=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum()),
                    reference_forward_span_m=float(end-start_x),reference_max_lateral_m=float(np.abs(y).max()),
                    reserved_width_m=float(2*half),
                    max_curvature=float(np.max(np.abs(curvature))),
                    guide_target_ground_xy=target.tolist(),footprint_checked=False,
                    unseen_near_field_m=float(start_x)))
        if best is not None:
            return best[1],"ok",best[2]
    return empty,"no_observed_image_corridor",{}
