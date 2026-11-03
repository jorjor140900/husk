"""Search of the observed green area for forward steering assistance, independent of edge fits.

The graph uses calibrated ground labels. Unknown/blocked cells cannot connect
branches. A route is smoothed only when its entire reserved width stays green.
This is a local planner, not a map of unseen parts of a route.
"""
import heapq
import math

import cv2
import numpy as np

from .vision_goal import VisionReferenceConfig


def _resample(path, step):
    lengths=np.linalg.norm(np.diff(path,axis=0),axis=1)
    path=path[np.r_[True,lengths>1e-7]]
    if len(path)<2:return path
    arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
    s=np.linspace(0,arc[-1],max(2,int(math.ceil(arc[-1]/step))+1))
    return np.column_stack([np.interp(s,arc,path[:,i]) for i in (0,1)])


def plan_area_reference(sample,start_x,config=VisionReferenceConfig()):
    """Dijkstra search + collision-checked smoothing on the green-area grid.

    Clearance removes cells too narrow for the configured robot width. Search
    can go around a non-drivable patch rather than score only fixed arcs.
    A long, central continuation supplies a local steering horizon. No goal,
    red boundary fits or waypoint queue is used.
    """
    empty=np.empty((0,2))
    if not math.isfinite(start_x) or not .05<=start_x<config.max_forward_m-.1:
        return empty,'near_axis_not_observable',{}
    step=.05;half=config.half_width_m+config.side_margin_m
    # Leave a short observed straight ingress before lateral steering. At the
    # bottom image row a turn normal can otherwise leave the field of view.
    ingress=.1
    calibrated_start=start_x
    offsets=np.linspace(-half,half,int(math.ceil(2*half/config.step_m))+1)
    # Search a bounded lateral start within the currently observed corridor.
    # Reject every candidate whose full ingress strip contains a veto. The
    # axis itself need not be the centre of the drivable corridor.
    ingress_detail={};seed_candidates=[]
    lateral_offsets=[0.,.05,-.05,.10,-.10,.15,-.15,.20,-.20]
    for advance in np.arange(0.,.301,.05):
        candidate=calibrated_start+advance
        for lateral in lateral_offsets:
            ix,iy=np.meshgrid(np.linspace(calibrated_start,candidate+ingress,
                int(math.ceil((advance+ingress)/config.step_m))+1),offsets+lateral,indexing='ij')
            entry=sample(np.column_stack([ix.ravel(),iy.ravel()])).reshape(ix.shape)
            if advance==0 and lateral==0:
                ingress_detail=dict(start_veto_samples=int((entry==0).sum()),start_unknown_samples=int((entry==255).sum()),
                    start_total_samples=int(entry.size),reserved_width_m=2*half,
                    start_veto_ground_xy=np.column_stack([ix[entry==0],iy[entry==0]]).round(3).tolist())
            if not np.any(entry==0) and np.all(entry[ix[:,0]>=candidate-1e-8]==1):
                seed_candidates.append(lateral)
        if seed_candidates:
            start_x=candidate;break
    else:
        return empty,'start_blocked_narrow_or_unobserved',ingress_detail
    start_x+=ingress
    # Keep the calibrated start exactly on the grid, including non-round mounts.
    xs=start_x+np.arange(-math.ceil(start_x/step),math.ceil((config.max_forward_m+half+.1-start_x)/step)+1)*step
    extent=max(config.max_lateral_m,2.)
    ys=np.arange(-math.ceil((extent+half+.1)/step),math.ceil((extent+half+.1)/step)+1)*step
    xx,yy=np.meshgrid(xs,ys,indexing='ij')
    labels=sample(np.column_stack([xx.ravel(),yy.ravel()])).reshape(xx.shape)
    # Match the existing supervised near-field contract: unseen pixels below
    # the calibrated start supply clearance, but known image vetoes still win.
    observed=(labels==1)|((labels==255)&(xx<start_x))
    clearance=cv2.distanceTransform(np.pad(observed.astype(np.uint8),1),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)[1:-1,1:-1]*step
    # Circular inflation previously rejected a green lateral strip because
    # unseen pixels behind it lay inside the circle. Use width across the
    # corridor here; the final path still checks its true normals in the mask.
    radius=int(math.ceil(half/step))
    lateral_clear=cv2.erode((labels==1).astype(np.uint8),np.ones((1,2*radius+1),np.uint8),
        borderType=cv2.BORDER_CONSTANT,borderValue=0)>0
    free=lateral_clear&(xx>=start_x-1e-8)&(xx<=config.max_forward_m+1e-8)&(abs(yy)<=extent)&(labels==1)
    start=None
    for lateral in seed_candidates:
        index=(int(np.argmin(abs(xs-start_x))),int(np.argmin(abs(ys-lateral))))
        if free[index]:start=index;break
    if start is None:return empty,'start_blocked_narrow_or_unobserved',dict(ingress_detail,start_lateral_width_rejected=True)
    start_y=float(ys[start[1]])
    distance=np.full(free.shape,np.inf);distance[start]=0.
    travel=np.zeros(free.shape)
    parents={};queue=[(0.,start)]
    moves=[(i,j,math.hypot(i,j)*step) for i in (-1,0,1) for j in (-1,0,1) if i or j]
    while queue:
        cost,(i,j)=heapq.heappop(queue)
        if cost>distance[i,j]:continue
        for di,dj,length in moves:
            ni,nj=i+di,j+dj
            if not (0<=ni<len(xs) and 0<=nj<len(ys) and free[ni,nj]):continue
            if di and dj and not (free[i,nj] and free[ni,j]):continue
            new=cost+length*(1.+.12/max(float(clearance[ni,nj])-half,.05))
            if new<distance[ni,nj]:
                distance[ni,nj]=new;travel[ni,nj]=travel[i,j]+length
                parents[(ni,nj)]=(i,j);heapq.heappush(queue,(new,(ni,nj)))
    reachable=np.isfinite(distance)
    # Seek a long, central continuation of the visible green corridor.
    # Straight branches win ties; there is no external destination.
    eligible=reachable&(np.hypot(xx-start_x,yy-start_y)>=config.min_length_m)
    if not eligible.any():return empty,'no_observed_image_corridor',{}
    score=np.full(free.shape,np.inf)
    score[reachable]=-travel[reachable]+.45/np.maximum(clearance[reachable]-half,.08)+.55*abs(yy[reachable])
    # A grid route can fail the finer normal-strip check even though another
    # reachable route fits. Keep spatially distinct alternatives instead of
    # treating failure of the single top-scoring endpoint as an empty corridor.
    endpoints=[]
    for flat in np.argsort(np.where(eligible,score,np.inf),axis=None):
        endpoint=np.unravel_index(flat,free.shape)
        if not eligible[endpoint]:break
        if all(math.hypot(xx[endpoint]-xx[e],yy[endpoint]-yy[e])>=.20 for e in endpoints):
            endpoints.append(endpoint)
        if len(endpoints)>=8:break
    offsets=np.linspace(-half,half,int(math.ceil(2*half/config.step_m))+1)
    def checked(points):
        delta=np.gradient(points,axis=0);norm=np.linalg.norm(delta,axis=1)
        if np.any(norm<1e-8):return None
        normal=np.column_stack([-delta[:,1],delta[:,0]])/norm[:,None]
        strip=(points[:,None,:]+normal[:,None,:]*offsets[None,:,None]).reshape(-1,2)
        # Sampling the entire strip again catches grid interpolation, unknown
        # pixels and sub-grid vetoes before any reference reaches the controller.
        if not np.all(sample(strip)==1):return None
        unit=delta/norm[:,None]
        curvature=np.linalg.norm(np.gradient(unit,axis=0),axis=1)/norm
        return float(curvature.max())
    best=None;attempts=0
    for endpoint in endpoints:
        attempts+=1
        cells=[endpoint]
        while cells[-1]!=start:cells.append(parents[cells[-1]])
        path=_resample(np.array([[xs[i],ys[j]] for i,j in reversed(cells)]),config.step_m)
        candidate_best=None
        for sigma_m in (.45,.30,.20,.10,.05,0.):
            points=path.copy()
            if sigma_m:
                pad=int(math.ceil(3*sigma_m/config.step_m))
                padded=np.pad(path,((pad,pad),(0,0)),mode='edge')
                points=cv2.GaussianBlur(padded,(1,0),sigmaX=0,sigmaY=sigma_m/config.step_m)[pad:-pad]
                points[0]=path[0];points[-1]=path[-1]
                points=_resample(points,config.step_m)
            curvature=checked(points)
            if curvature is None or curvature>4.:continue
            if candidate_best is None or curvature<candidate_best[0]:candidate_best=(curvature,points)
        if candidate_best is not None:
            best=candidate_best;break
    # A turning route can lose its outer strip at the image boundary. A fully
    # observed straight horizon is still a useful control reference. Check all
    # seed strips, from longest to shortest, with the SAME exact width check.
    if best is None:
        for length in np.arange(config.max_forward_m-start_x,config.min_length_m-1e-8,-.10):
            for lateral in seed_candidates:
                points=_resample(np.array([[start_x,lateral],[start_x+length,lateral]]),config.step_m)
                curvature=checked(points)
                if curvature is not None:
                    best=(curvature,points);break
            if best is not None:break
    if best is None:
        return empty,'no_observed_image_corridor',dict(area_candidate_routes_checked=attempts)
    curvature,path=best
    return path,'ok',dict(reference_kind='green_area_assist',reference_start_m=float(path[0,0]),reference_start_lateral_m=float(path[0,1]),
        area_candidate_routes_checked=attempts,
        observed_reference_length_m=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()),
        reference_forward_span_m=float(path[-1,0]-path[0,0]),reference_max_lateral_m=float(abs(path[:,1]).max()),
        reserved_width_m=2*half,max_curvature=curvature,
        reference_end_ground_xy=path[-1].tolist(),goal_priority=False,
        footprint_checked=False,unseen_near_field_m=float(start_x),
        red_edges_used_for_planning=False,observed_ingress_m=ingress,calibrated_start_m=float(calibrated_start),start_advance_m=float(start_x-ingress-calibrated_start),area_search_reachable_cells=int(reachable.sum()))
