"""Experimental local image/scan-plane fusion, without ROS or global costmaps.

Finite measured rays support free cells in the scan plane only. Missing rays
remain unknown. This cannot certify ground, overhangs, scan deskew or braking.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np

from .collision import CollisionConfig, stop_sweep
from .navigation import mapping_scan_geometry


@dataclass(frozen=True)
class LocalFusionConfig:
    front_m: float = .5
    rear_m: float = .56
    half_width_m: float = .33
    margin_m: float = .15
    side_margin_m: float = .08
    resolution_m: float = .05
    extent_m: float = 4.5
    lidar_min_range_m: float = .1
    assume_near_field_clear: bool = False

    def __post_init__(self):
        if type(self.assume_near_field_clear) is not bool:
            raise ValueError("near_field_assumption_must_be_boolean")
        if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0
               for k,v in self.__dict__.items() if k!='assume_near_field_clear'):
            raise ValueError("invalid_local_fusion_dimensions")
        if not .025<=self.resolution_m<=.05 or not 3.5<=self.extent_m<=6:
            raise ValueError("invalid_local_fusion_grid")
        if max(self.front_m,self.rear_m,self.half_width_m)>1.5:
            raise ValueError("unsupported_local_fusion_hull")


def planar(matrix):
    m=np.asarray(matrix,float)
    if (m.shape!=(4,4) or not np.isfinite(m).all() or
            not np.allclose(m[3],[0,0,0,1],atol=1e-8) or
            not np.allclose(m[:3,:3].T@m[:3,:3],np.eye(3),atol=1e-5) or
            not np.isclose(np.linalg.det(m[:3,:3]),1,atol=1e-5) or abs(m[2,2]-1)>.001):
        raise ValueError("nonplanar_local_lidar_transform")
    return m


def transform_xy(points,matrix):
    m=planar(matrix)
    return np.asarray(points,float)@m[:2,:2].T+m[:2,3]


def local_goal_target(goal,lookahead=2.5,*,allow_side_goal=False):
    goal=np.asarray(goal,float)
    if goal.shape!=(2,) or not np.isfinite(goal).all() or np.linalg.norm(goal)<.05:
        raise ValueError("local_goal_not_ahead")
    if not allow_side_goal and goal[0]<=.05:raise ValueError("local_goal_not_ahead")
    if abs(math.atan2(goal[1],goal[0]))>math.radians(110 if allow_side_goal else 65):
        raise ValueError("local_goal_requires_reorientation")
    return goal*min(1.,lookahead/np.linalg.norm(goal))


def validate_local_scan(scan):
    geometry=mapping_scan_geometry(scan)
    if scan.time_increment!=0:raise ValueError("local_scan_requires_deskew")
    if (abs(scan.angle_max-geometry['angle_max'])>2*scan.angle_increment or
            abs(len(scan.ranges)*scan.angle_increment-2*math.pi)>1.1*scan.angle_increment):
        raise ValueError("unsupported_local_scan_geometry")
    ranges=np.asarray(scan.ranges,float)
    if np.any(ranges[np.isfinite(ranges)]<=0):raise ValueError("invalid_local_scan_zero_range")
    return geometry


def validate_near_field_assumption(enabled,local_fusion,supervised_trial):
    if any(type(value) is not bool for value in (enabled,local_fusion,supervised_trial)):
        raise ValueError("near_field_assumption_flags_must_be_boolean")
    if enabled and not (local_fusion and supervised_trial):
        raise ValueError("assume_near_field_clear requires local_fusion and supervised_trial")


class LocalLidarGrid:
    """One original scan snapshot in base coordinates at its capture time.

    Rebuilt from observations, not accumulated in map. A disappeared return
    becomes unknown unless another finite ray supplies free-space evidence.
    Geometry correction is mapping-only and never changes the source timestamp.
    """
    def __init__(self,scan,base_from_sensor,config=LocalFusionConfig()):
        self.config=config
        geometry=validate_local_scan(scan)
        matrix=planar(base_from_sensor)
        if np.linalg.norm(matrix[:2,3])>2:
            raise ValueError("local_lidar_origin_outside_robot_geometry")
        self.res=config.resolution_m
        self.origin=np.array([-config.extent_m,-config.extent_m])
        self.size=int(math.ceil(2*config.extent_m/self.res))+1
        free=np.zeros((self.size,self.size),np.uint8)
        occupied=np.zeros_like(free)
        ranges=np.asarray(scan.ranges,float)
        minimum=max(config.lidar_min_range_m,scan.range_min)
        if minimum>=config.extent_m:
            raise ValueError("local_lidar_blind_range_exceeds_grid")
        # Keep ALL reported positive hits, including unexpected near returns.
        # The sensor minimum limits clearing evidence, never obstacle marking.
        indices=np.flatnonzero(np.isfinite(ranges)&(ranges>0))
        angle=scan.angle_min+indices*scan.angle_increment
        directions=np.column_stack([np.cos(angle),np.sin(angle)])
        # Finite range clipping shortens a measured free ray. +inf never enters
        # this set, and cannot turn into a clearing ray at the sensor maximum.
        can_clear=ranges[indices]>=minimum
        lengths=np.minimum(ranges[indices][can_clear],config.extent_m)
        starts=transform_xy(directions[can_clear]*minimum,matrix)
        ends=transform_xy(directions[can_clear]*lengths[:,None],matrix)
        hits=transform_xy(directions*ranges[indices,None],matrix)
        def pixel(point):
            p=np.floor((point-self.origin)/self.res).astype(int)
            return int(p[0]),int(p[1])
        if len(starts):
            # Visit every cell intersected by a measured ray. Pixel Bresenham
            # omits some crossed cells for a fractionally offset sensor, making
            # artificial unknown holes even in a densely observed empty room.
            # Split at exact grid-line intersections; never dilate free space.
            boundaries=self.origin[0]+np.arange(self.size+1)*self.res
            delta=ends-starts
            with np.errstate(divide='ignore',invalid='ignore'):
                tx=(boundaries[None,:]-starts[:,0,None])/delta[:,0,None]
                ty=(boundaries[None,:]-starts[:,1,None])/delta[:,1,None]
            tx=np.where((tx>0)&(tx<1),tx,1.)
            ty=np.where((ty>0)&(ty<1),ty,1.)
            edges=np.sort(np.column_stack([np.zeros(len(starts)),tx,ty,np.ones(len(starts))]),axis=1)
            mid=(edges[:,:-1]+edges[:,1:])/2
            used=edges[:,1:]-edges[:,:-1]>1e-12
            rows,columns=np.nonzero(used)
            points=starts[rows]+delta[rows]*mid[rows,columns,None]
            cells=np.floor((points-self.origin)/self.res).astype(int)
            inside=(cells>=0).all(axis=1)&(cells<self.size).all(axis=1)
            free[cells[inside,1],cells[inside,0]]=1
        for hit,length in zip(hits,ranges[indices]):
            if length<=config.extent_m:
                x,y=pixel(hit)
                if 0<=x<self.size and 0<=y<self.size:
                    occupied[y,x]=1
        # Only the measured physical hull is known to be occupied by the robot.
        # Do not fill a padded exclusion circle or its unseen near field.
        axis=self.origin[0]+(np.arange(self.size)+.5)*self.res
        xx,yy=np.meshgrid(axis,axis)
        body=((xx-self.res/2>=-config.rear_m)&(xx+self.res/2<=config.front_m)&
              (np.abs(yy)+self.res/2<=config.half_width_m))
        free[body]=1;occupied[body]=0
        free[occupied>0]=0
        self.free=free
        self.labels=np.where(occupied,0,np.where(free,1,255)).astype(np.uint8)
        self.points=hits[ranges[indices]<=config.extent_m]
        self.finite_rays=len(indices)
        self.unknown_rays=int(np.isposinf(ranges).sum())
        self.metadata_normalized=bool(scan.range_min!=geometry['range_min'] or scan.angle_max!=geometry['angle_max'])
        self.cache={}
        self.discretization_extra_m=math.hypot(max(config.front_m,config.rear_m)+config.margin_m,
            config.half_width_m+config.side_margin_m)*math.sin(math.radians(2.5))+self.res*math.sqrt(2)
        pad=self.discretization_extra_m+self.res
        self.near_field_bounds=(-config.rear_m-config.margin_m-pad,config.front_m+config.margin_m+pad,
            -config.half_width_m-config.side_margin_m-pad,config.half_width_m+config.side_margin_m+pad)
        lo,hi,right,left=self.near_field_bounds
        near=((xx-self.res/2>=lo)&(xx+self.res/2<=hi)&
              (yy-self.res/2>=right)&(yy+self.res/2<=left))
        self.assumed_unknown=(self.labels==255)&near&config.assume_near_field_clear
        self.assumed_unknown_cells=int(self.assumed_unknown.sum())
        # Keep raw labels and published evidence truthful. The operator's
        # bounded assumption is separate support for footprint checks only.
        self.pose_support=self.free.copy()
        self.pose_support[self.assumed_unknown]=1
        # Coverage is not clearance: occupied cells are observed, but all hits
        # outside the physical hull are checked separately in exact coordinates.
        # This avoids adding grid rounding inflation twice to real obstacles.
        self.pose_coverage=((self.labels!=255)|self.assumed_unknown).astype(np.uint8)
        points=self.points
        own=((points[:,0]>=-config.rear_m)&(points[:,0]<=config.front_m)&
             (np.abs(points[:,1])<=config.half_width_m))
        self.collision_points=points[~own]
        self.clearing_rays=int(can_clear.sum())

    def sample(self,points):
        p=np.asarray(points,float).reshape(-1,2)
        coords=(p-self.origin)/self.res-.5
        result=np.full(len(p),255,np.uint8)
        inside=np.isfinite(coords).all(axis=1)&(coords>=0).all(axis=1)&(coords<=self.size-1).all(axis=1)
        ids=np.flatnonzero(inside)
        low=np.floor(coords[ids]).astype(int);high=np.ceil(coords[ids]).astype(int)
        neighbours=np.array([self.labels[y,x] for x,y in (low.T,high.T,
            np.column_stack([low[:,0],high[:,1]]).T,np.column_stack([high[:,0],low[:,1]]).T)])
        result[ids[np.all(neighbours==1,axis=0)]]=1
        result[ids[np.any(neighbours==0,axis=0)]]=0
        return result

    def clear_poses(self,poses,point_padding_m=.01):
        poses=np.asarray(poses,float)
        if poses.ndim!=2 or poses.shape[1]!=3 or not np.isfinite(poses).all():
            raise ValueError("invalid_local_collision_poses")
        if not math.isfinite(point_padding_m) or point_padding_m<0:
            raise ValueError("invalid_local_point_sampling_padding")
        cfg=self.config
        if len(self.collision_points):
            for x,y,yaw in poses:
                c,s=math.cos(yaw),math.sin(yaw)
                local=(self.collision_points-[x,y])@np.array([[c,-s],[s,c]])
                inside=((local[:,0]<=cfg.front_m+cfg.margin_m+point_padding_m)&
                    (local[:,0]>=-cfg.rear_m-cfg.margin_m-point_padding_m)&
                    (np.abs(local[:,1])<=cfg.half_width_m+cfg.side_margin_m+point_padding_m))
                if inside.any():return False
        # Conservative orientation and cell rounding bounds, while retaining
        # the measured rectangular hull rather than a large enclosing circle.
        quantum=math.radians(5)
        bins=np.rint(poses[:,2]/quantum).astype(int)
        radius=math.hypot(max(cfg.front_m,cfg.rear_m)+cfg.margin_m,cfg.half_width_m+cfg.side_margin_m)
        error=self.discretization_extra_m
        span=int(math.ceil((radius+error)/self.res))
        offsets=np.arange(-span,span+1)*self.res
        xx,yy=np.meshgrid(offsets,offsets)
        for index in np.unique(bins):
            key=int(index)%72
            if key not in self.cache:
                yaw=key*quantum;c,s=math.cos(yaw),math.sin(yaw)
                bx=c*xx+s*yy;by=-s*xx+c*yy
                kernel=((bx<=cfg.front_m+cfg.margin_m+error)&(bx>=-cfg.rear_m-cfg.margin_m-error)&
                        (np.abs(by)<=cfg.half_width_m+cfg.side_margin_m+error)).astype(np.uint8)
                support=self.pose_coverage if cfg.assume_near_field_clear else self.pose_support
                self.cache[key]=cv2.erode(support,kernel,borderType=cv2.BORDER_CONSTANT,borderValue=0)
            valid=self.cache[key]
            xy=(poses[bins==index,:2]-self.origin)/self.res-.5
            if ((xy<0).any() or (xy>self.size-1).any()):
                return False
            low=np.floor(xy).astype(int);high=np.ceil(xy).astype(int)
            for x,y in (low.T,high.T,np.column_stack([low[:,0],high[:,1]]).T,
                        np.column_stack([high[:,0],low[:,1]]).T):
                if not valid[y,x].all():return False
        return True

    def reference_clear(self,path,scan_base_from_ground,start_x):
        path=np.asarray(path,float)
        if len(path)<2 or np.linalg.norm(path[0]-[start_x,0.])>1e-6:return False
        # Validate only the observed reference here. Do not invent a straight
        # connector over the camera's near blind field. Actual commands must
        # separately pass command_clear() from the robot's current footprint.
        yaw=np.arctan2(np.gradient(path[:,1]),np.gradient(path[:,0]))
        m=planar(scan_base_from_ground)
        transformed=transform_xy(path,m)
        radius=math.hypot(max(self.config.front_m,self.config.rear_m)+self.config.margin_m,
            self.config.half_width_m+self.config.side_margin_m)
        # For linear interpolation of sampled poses, every body corner is at
        # most half this per-segment displacement from one endpoint sample.
        motion=np.linalg.norm(np.diff(transformed,axis=0),axis=1)+radius*np.abs(np.diff(np.unwrap(yaw)))
        padding=max(.01,float(motion.max())/2)
        return self.clear_poses(np.column_stack([transformed,yaw+math.atan2(m[1,0],m[0,0])]),padding)

    def command_clear(self,requested,measured,scan_base_from_current_base,age,limits):
        if self.finite_rays==0:return False
        config=CollisionConfig(front_m=self.config.front_m,rear_m=self.config.rear_m,
            half_width_m=self.config.half_width_m,margin_m=self.config.margin_m,
            lateral_margin_m=self.config.side_margin_m,max_linear_mps=limits[0],max_angular_rps=limits[1])
        poses=np.vstack([stop_sweep(v,config,extra_reaction_sec=age) for v in (requested,measured)])
        matrix=planar(scan_base_from_current_base)
        points=transform_xy(poses[:,:2],matrix)
        poses=np.column_stack([points,poses[:,2]+math.atan2(matrix[1,0],matrix[0,0])])
        return self.clear_poses(poses,config.sweep_step_m)


def fused_sampler(image_sample,grid,scan_base_from_ground):
    def sample(points):
        image=image_sample(points)
        lidar=grid.sample(transform_xy(points,scan_base_from_ground))
        return np.where((image==0)|(lidar==0),0,np.where((image==1)&(lidar==1),1,255)).astype(np.uint8)
    return sample
