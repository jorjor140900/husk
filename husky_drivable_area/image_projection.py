"""Project a metric controller reference onto the calibrated camera image."""
import cv2
import numpy as np
from .control import validate_path
from .ground_geometry import ground_to_image

def project_reference_path(points_xy, path_to_ground, calibration, mount):
    """Map/odom planar XY -> measured ground plane -> raw camera pixels.

    Reference paths are 2D. Their XY is transformed into the ground frame and
    placed on its z=0 plane, rather than treating the odometry base height as
    floor height. No connector from the robot to an off-path point is invented.
    """
    points = validate_path(points_xy, .5)
    tf = np.asarray(path_to_ground, float)
    if (tf.shape != (4,4) or not np.isfinite(tf).all() or
            not np.allclose(tf[3], [0,0,0,1]) or
            not np.allclose(tf[:3,:3].T@tf[:3,:3],np.eye(3),atol=1e-6) or
            abs(np.linalg.det(tf[:3,:3])-1)>1e-6 or abs(tf[2,2]-1)>1e-3):
        raise ValueError("plan_requires_planar_capture_tf")
    points = points@tf[:2,:2].T+tf[:2,3]
    # Show the route from the nearest projection onward, not already-passed
    # portions of the global plan behind the robot.
    segments = np.diff(points,axis=0)
    lengths2 = np.sum(segments**2,axis=1)
    fractions = np.clip(-np.sum(points[:-1]*segments,axis=1)/lengths2,0,1)
    nearest = points[:-1]+fractions[:,None]*segments
    index = int(np.argmin(np.sum(nearest**2,axis=1)))
    ahead = np.vstack([nearest[index],points[index+1:]])
    lengths = np.linalg.norm(np.diff(ahead,axis=0),axis=1)
    ahead = ahead[np.r_[True,lengths>1e-6]]
    if len(ahead)<2:
        return np.empty((0,2))
    arc = np.r_[0,np.cumsum(np.linalg.norm(np.diff(ahead,axis=0),axis=1))]
    end = min(float(arc[-1]),8.)
    distances = np.r_[np.arange(0,end,.025),end]
    ground = np.column_stack([np.interp(distances,arc,ahead[:,i]) for i in (0,1)])
    raw, visible = ground_to_image(ground,calibration,mount)
    indices = np.flatnonzero(visible)
    if len(indices):
        camera = (np.column_stack([ground[indices],np.zeros(len(indices))])-mount.origin)@mount.rotation
        expected = camera[:,:2]/camera[:,2,None]
        recovered = cv2.undistortPoints(raw[indices].reshape(-1,1,2),calibration.matrix,
                                      calibration.distortion).reshape(-1,2)
        error = np.linalg.norm((expected-recovered)*[calibration.matrix[0,0],calibration.matrix[1,1]],axis=1)
        visible[indices] &= np.isfinite(error)&(error<=.5)
    raw[~visible] = np.nan
    return raw
