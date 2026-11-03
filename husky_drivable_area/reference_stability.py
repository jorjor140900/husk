"""Reuse geometry only after validating its visible strip in the NEW capture."""
import math
import numpy as np


def reusable_reference(points,sample,start_x,half_width,step=.025,*,allow_unknown=False,truncate_invalid=False):
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=2 or len(points)<2 or not np.isfinite(points).all():return None
    visible=np.flatnonzero(points[:,0]>=start_x)
    if not len(visible):return None
    path=points[visible[0]:]
    if len(path)<2 or np.linalg.norm(np.diff(path,axis=0),axis=1).sum()<.15:return None
    # Resample continuously: sparse old points must not jump a new veto.
    lengths=np.linalg.norm(np.diff(path,axis=0),axis=1);arc=np.r_[0,np.cumsum(lengths)]
    if (lengths<=1e-8).any():return None
    s=np.linspace(0,arc[-1],int(math.ceil(arc[-1]/step))+1)
    path=np.column_stack([np.interp(s,arc,path[:,i]) for i in (0,1)])
    delta=np.gradient(path,axis=0);norm=np.linalg.norm(delta,axis=1)
    normals=np.column_stack([-delta[:,1],delta[:,0]])/norm[:,None]
    offsets=np.linspace(-half_width,half_width,int(math.ceil(2*half_width/step))+1)
    strip=(path[:,None,:]+normals[:,None,:]*offsets[None,:,None]).reshape(-1,2)
    labels=sample(strip)
    if truncate_invalid and not allow_unknown:
        # Keep only the contiguous, currently observed prefix. Never skip a
        # bad section to join a later green section or relabel a veto.
        valid=(labels.reshape(len(path),len(offsets))==1).all(axis=1)
        failed=np.flatnonzero(~valid)
        if len(failed):
            end=int(failed[0])-1  # leave one sample before the failing strip
            prefix=path[:max(0,end)]
            if len(prefix)<2 or np.linalg.norm(np.diff(prefix,axis=0),axis=1).sum()<.5:return None
            # Trimming changes the endpoint normal: check it again.
            return reusable_reference(prefix,sample,start_x,half_width,step)
    if allow_unknown:
        if not np.isin(labels,[1,255]).all():return None
    elif not np.all(labels==1):return None
    return path


def choose_stable_reference(new,previous,target,slack=.10):
    if previous is None:return new,False
    if not len(new) or np.linalg.norm(previous[-1]-target)<=np.linalg.norm(new[-1]-target)+slack:
        return previous,True
    return new,False


class EdgeSmoother:
    """Display-only EMA of currently supported fits; absent fits stay absent."""
    def __init__(self):self.previous=None;self.stamp=0;self.shape=None

    def update(self,edges,stamp,shape):
        from dataclasses import replace
        if stamp==self.stamp and shape==self.shape:return self.previous
        fresh=self.previous is not None and self.shape==shape and 0<stamp-self.stamp<=250_000_000
        result=[]
        for i,edge in enumerate(edges):
            old=self.previous[i] if fresh else None
            if edge is not None and old is not None:
                rows=[edge.y_top,edge.y_bottom]
                if max(abs(edge.x_at(y)-old.x_at(y)) for y in rows)<shape[1]*.06:
                    edge=replace(edge,slope=.4*edge.slope+.6*old.slope,intercept=.4*edge.intercept+.6*old.intercept)
            result.append(edge)
        self.previous=result;self.stamp=stamp;self.shape=shape
        return result
