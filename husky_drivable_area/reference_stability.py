"""Reuse geometry only after checking its visible strip in the current capture."""
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
