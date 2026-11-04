"""Ground-label sampling and reference dimensions used by camera assist."""
from dataclasses import dataclass
import math
import cv2
import numpy as np
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
