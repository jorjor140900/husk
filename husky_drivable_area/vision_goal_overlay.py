"""The exact controller reference on the exact source image; no second planner."""
import cv2
import numpy as np

from .ground_overlay import robot_axis_pixels
from .nav2_camera_path import project_nav2_path


def draw_vision_reference(frame,labels,reference,camera,mount,reason):
    image=frame.copy()
    mask=cv2.resize((labels==1).astype(np.uint8),(frame.shape[1],frame.shape[0]),interpolation=cv2.INTER_NEAREST)>0
    green=np.full_like(frame,(0,200,0))
    image[mask]=cv2.addWeighted(frame,.65,green,.35,0)[mask]
    contours,_=cv2.findContours(mask.astype(np.uint8)*255,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image,contours,-1,(255,220,0),1)
    axis=robot_axis_pixels(camera,mount,frame.shape[:2],frame.shape[:2],mount.camera_frame)
    paths=[(axis,(255,0,0),3)]
    if len(reference)>=2:
        paths.append((project_nav2_path(reference,np.eye(4),camera,mount),(0,255,255),2))
    for points,colour,width in paths:
        finite=np.isfinite(points).all(axis=1)
        edges=np.flatnonzero(np.diff(np.r_[False,finite,False].astype(int)))
        for start,end in zip(edges[::2],edges[1::2]):
            if end-start>=2:
                cv2.polylines(image,[np.rint(points[start:end]).astype(np.int32)],False,colour,width,cv2.LINE_AA)
    cv2.rectangle(image,(0,0),(image.shape[1],65),(20,20,20),-1)
    for y,text in ((18,"IMAGE + AI: "+reason),(38,"Yellow=controller reference  Blue=robot Y=0"),
                   (58,"Goal selects direction | downstream safety required")):
        cv2.putText(image,text,(7,y),cv2.FONT_HERSHEY_SIMPLEX,.42,(255,255,255),1)
    return image
