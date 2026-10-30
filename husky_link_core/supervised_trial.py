"""Explicit attended trial limits; never marks field measurements passed."""
import math
from .protocol import finite, frame_id, stamp_ns

MODE = 'supervised_vision_trial'
LIMITS = dict(max_linear_mps=.03, max_angular_rps=.15, max_distance_m=.5,
              max_duration_sec=20., max_rotation_rad=.35)
GOAL_LIMITS = dict(LIMITS, max_linear_mps=.05, max_distance_m=3., max_duration_sec=120.)

EXTENDED_GOAL_LIMITS = dict(GOAL_LIMITS,max_linear_mps=.15,max_distance_m=100.,max_duration_sec=300.)
LONG_GOAL_LIMITS = dict(EXTENDED_GOAL_LIMITS,max_duration_sec=900.)


def selected_limits(goal=False,extended=False,duration=300.):
    if type(goal) is not bool or type(extended) is not bool or extended and not goal:
        raise ValueError('extended session requires explicit goal session')
    if type(duration) not in (int,float) or duration not in (300.,900.):
        raise ValueError('extended session duration must be 300 or 900 seconds')
    return (LONG_GOAL_LIMITS if duration==900. else EXTENDED_GOAL_LIMITS).copy() if extended else (GOAL_LIMITS if goal else LIMITS).copy()


def checked_limits(value):
    if not isinstance(value,dict) or value not in (LIMITS,GOAL_LIMITS,EXTENDED_GOAL_LIMITS,LONG_GOAL_LIMITS):
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
