"""Explicit attended trial limits; never marks field measurements passed."""
import math
from .protocol import finite, frame_id, stamp_ns

MODE = 'supervised_vision_trial'
LIMITS = dict(max_linear_mps=.03, max_angular_rps=.15, max_distance_m=.5,
              max_duration_sec=20., max_rotation_rad=.35)
GOAL_LIMITS = dict(LIMITS, max_linear_mps=.05, max_distance_m=3., max_duration_sec=120.)

EXTENDED_GOAL_LIMITS = dict(GOAL_LIMITS,max_linear_mps=.15,max_distance_m=100.,max_duration_sec=300.)
LONG_GOAL_LIMITS = dict(EXTENDED_GOAL_LIMITS,max_duration_sec=900.)
TURN_GOAL_LIMITS = dict(LONG_GOAL_LIMITS,max_rotation_rad=2*math.pi)
CAMERA_ASSIST_LIMITS = dict(TURN_GOAL_LIMITS,max_linear_mps=.18)
CAMERA_ASSIST_FAST_LIMITS = dict(CAMERA_ASSIST_LIMITS,max_linear_mps=.234)


# These conditions invalidate the reference, not the operator's session.
# Keep the list explicit: permission, odometry, transport, and budget faults
# must never be turned into automatic recovery by a broad "vision" match.
VISION_PAUSE_REASONS = frozenset((
    'start_blocked_narrow_or_unobserved', 'near_axis_not_observable',
    'no_observed_image_corridor', 'area_turn_too_sharp', 'reference_exhausted',
    'target_not_ahead', 'too_far_from_path', 'waiting_for_image',
    'stale_segmentation', 'waiting_for_capture_tf', 'image_reference_stale',
    'capture_expired_waiting_for_tf', 'vision_planning_too_old',
    'camera_timeout', 'stale_input', 'stale_result', 'stale_output',
    'stale_segmentation_before_preview',
))
CAMERA_PACKET_PAUSE_REASONS = frozenset((
    'invalid_segmentation_packet', 'invalid_image_geometry', 'invalid_path',
))


def recoverable_vision_gap(enabled,reason,navigation):
    """Pause at zero, within an existing operator session; never re-arm."""
    if (enabled is not True or not isinstance(navigation,dict) or
            navigation.get('enabled') is not True or navigation.get('allow_motion') is not True):
        return False
    camera = navigation.get('mode') == 'camera_steering_assist'
    # A missing gate heartbeat pauses camera assist while request/permission/
    # odometry remain fresh (checked earlier by the caller). A lost control
    # process still trips the separate request watchdog.
    if reason == 'trial_navigation_gate_stale':return camera
    if reason != 'trial_navigation_gate_closed':return False
    gate_reason=navigation.get('reason')
    vision_reason=navigation.get('vision_reason')
    if not isinstance(vision_reason,str):return False
    if gate_reason not in ('vision_invalid','vision_stale'):return False
    return (vision_reason in VISION_PAUSE_REASONS or
            camera and vision_reason in CAMERA_PACKET_PAUSE_REASONS or
            gate_reason=='vision_stale' and vision_reason=='tracking')


def selected_limits(goal=False,extended=False,duration=300.,*,turning=False,camera_assist=False,camera_assist_speed=.18):
    if type(goal) is not bool or type(extended) is not bool or extended and not goal:
        raise ValueError('extended session requires explicit goal session')
    if type(duration) not in (int,float) or duration not in (300.,900.):
        raise ValueError('extended session duration must be 300 or 900 seconds')
    if type(turning) is not bool or turning and not (goal and extended and duration==900.):
        raise ValueError('turning session requires extended goal and 900 seconds')
    if type(camera_assist) is not bool or camera_assist and not turning:
        raise ValueError('camera assist preset requires turning session')
    if (type(camera_assist_speed) not in (int,float) or camera_assist_speed not in (.18,.234)
            or not camera_assist and camera_assist_speed!=.18):
        raise ValueError('camera assist speed requires the 0.18 or 0.234 preset')
    if camera_assist:return (CAMERA_ASSIST_FAST_LIMITS if camera_assist_speed==.234 else CAMERA_ASSIST_LIMITS).copy()
    if turning:return TURN_GOAL_LIMITS.copy()
    return (LONG_GOAL_LIMITS if duration==900. else EXTENDED_GOAL_LIMITS).copy() if extended else (GOAL_LIMITS if goal else LIMITS).copy()


def checked_limits(value):
    if not isinstance(value,dict) or value not in (LIMITS,GOAL_LIMITS,EXTENDED_GOAL_LIMITS,LONG_GOAL_LIMITS,TURN_GOAL_LIMITS,CAMERA_ASSIST_LIMITS,CAMERA_ASSIST_FAST_LIMITS):
        raise ValueError('supervised trial limits must match a fixed bounded configuration')
    return value.copy()


def profile_trial_limits(profile):
    return checked_limits(profile.get('trial_limits'))


def trial_mode(profile, requested=False):
    if not isinstance(profile, dict):
        raise ValueError('profile must be a JSON object')
    if type(requested) is not bool:
        raise ValueError('supervised_trial must be bool')
    declared = profile.get('operation_mode')
    if declared not in (None, 'field', MODE):
        raise ValueError('unknown operation_mode')
    if requested != (declared == MODE):
        raise ValueError('trial profile and explicit supervised_trial flag must match')
    if requested:profile_trial_limits(profile)
    return requested


def bounded_command(v, w, limits=None):
    limits=checked_limits(LIMITS if limits is None else limits)
    if not finite(v) or not finite(w) or not 0 <= v <= max(.05,limits['max_linear_mps'])+.000001 or abs(w) > .200001:
        raise ValueError('invalid_trial_request')
    scale = min(1., limits['max_linear_mps']/max(v,1e-12),
                limits['max_angular_rps']/max(abs(w),1e-12))
    return v*scale, w*scale


def odometry_sample(message, odom, base):
    if frame_id(message.header.frame_id) != odom or frame_id(message.child_frame_id) != base:
        raise ValueError('trial_odometry_frame_mismatch')
    stamp=message.header.stamp
    ns=stamp_ns(dict(secs=getattr(stamp,'sec',getattr(stamp,'secs',None)),
                     nsecs=getattr(stamp,'nanosec',getattr(stamp,'nsecs',None))))
    p,q=message.pose.pose.position,message.pose.pose.orientation
    if not all(finite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):
        raise ValueError('invalid_trial_odometry')
    norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
    if norm<1e-12 or 2*(q.x*q.x+q.y*q.y)/norm>.001:
        raise ValueError('nonplanar_trial_odometry')
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y)/norm,1-2*(q.y*q.y+q.z*q.z)/norm)
    return p.x,p.y,yaw,ns


class TrialBudget(object):
    def __init__(self, limits=None):
        self.limits=checked_limits(LIMITS if limits is None else limits)
        self.pose=None
        self.active=False
        self.fault=''
        self.started=0.
        self.distance=self.rotation=0.

    def observe(self, sample, received):
        if self.pose is not None:
            old=self.pose[0]
            if sample[3] <= old[3]:
                if sample[3] < old[3] and self.active:self.stop('trial_odometry_reversed')
                return
            if self.active:
                self.distance+=math.hypot(sample[0]-old[0],sample[1]-old[1])
                delta=sample[2]-old[2]
                self.rotation+=abs(math.atan2(math.sin(delta),math.cos(delta)))
        self.pose=(sample,received)

    def reset(self, now):
        self.active=True
        self.fault=''
        self.started=now
        self.distance=self.rotation=0.

    def stop(self, reason):
        if not self.fault:self.fault=reason
        self.active=False

    def check(self, now, ros_ns):
        if self.fault:return self.fault
        reason=''
        if self.pose is None:reason='trial_no_odometry'
        elif not -.05 <= (ros_ns-self.pose[0][3])/1e9 <= .3 or not 0 <= now-self.pose[1] <= .3:
            reason='trial_odometry_stale'
        if self.active:
            if now < self.started:reason='trial_clock_reversed'
            elif now-self.started >= self.limits['max_duration_sec']:reason='trial_time_limit'
            elif self.distance >= self.limits['max_distance_m']:reason='trial_distance_limit'
            elif self.rotation >= self.limits['max_rotation_rad']:reason='trial_rotation_limit'
        if reason and self.active:self.stop(reason)
        return reason or 'clear'
