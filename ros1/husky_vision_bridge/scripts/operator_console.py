#!/usr/bin/env python3
"""Local operator START/STOP for AI autonomy. Never publishes a Twist."""
import json
import os
import select
import signal
import sys
import termios
import time
import tty

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from husky_link_core.operator_permission import OperatorSession, operator_settings
from husky_link_core.protocol import loads
from husky_link_core.supervised_trial import trial_mode, LIMITS


def main():
    rospy.init_node("operator_console", disable_signals=True)
    if not sys.stdin.isatty():
        raise RuntimeError("Interactive terminal required; use ssh -t. Do not run under roslaunch/nohup.")
    with open(os.path.expanduser(rospy.get_param("~profile_path"))) as source:
        profile = json.load(source)
    allow = rospy.get_param("~allow_robot_output", False)
    trial=trial_mode(profile,rospy.get_param("~supervised_trial",False))
    topic, expected = operator_settings(profile, allow, trial)
    duration=rospy.get_param("~session_sec",30.0)
    if trial:
        duration=min(float(duration),LIMITS['max_duration_sec'])
        print("SUPERVISED TRIAL: LiDAR NOT CHECKED. Operator must watch robot and keep hardware e-stop available.")
    session = OperatorSession(duration, expected, allow)
    # A single console owns this local permission topic. No latching/background
    # auto-enable; terminal/process loss lets the gateway's .3s deadline expire.
    publisher = rospy.Publisher(topic, Bool, queue_size=1, latch=False)
    def received(message):
        try:
            value = loads(message.data)
        except (ValueError, TypeError, OverflowError):
            value = None
        session.receive_state(value, time.monotonic())
    subscriber = rospy.Subscriber("/husky_vision/state", String, received, queue_size=1)
    rospy.wait_for_service("/husky_vision/set_enabled", timeout=5)
    gateway = rospy.ServiceProxy("/husky_vision/set_enabled", SetBool)
    def disarm():
        publisher.publish(Bool(data=False))
        try:
            gateway(False)
        except rospy.ServiceException:
            pass  # The independent permission deadline still expires locally.
    def shutdown(signum, frame):
        rospy.signal_shutdown("operator terminal closed or interrupted")
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, shutdown)
    print("AI autonomy permission: g=START one %.0fs session (no Enter); Space/any other key=STOP; q=STOP+exit." % session.duration)
    print("Output: %s; driver enabled: %s. Enable the ROS2 link BEFORE pressing g." % (expected, allow))
    print("This console grants permission; the image controller supplies steering. No automatic restart.")
    settings = termios.tcgetattr(sys.stdin.fileno())
    previous_phase = None
    try:
        tty.setcbreak(sys.stdin.fileno())
        disarm()
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], .05)[0]:
                key = sys.stdin.read(1)
                if key == "g":
                    accepted = session.start(time.monotonic(), rospy.Time.now().to_nsec())
                    print("START accepted" if accepted else "START ignored: " + session.reason, flush=True)
                else:
                    session.stop("terminal_eof" if not key else "operator_stop")
                    disarm()
                    if not key or key == "q":
                        break
            permitted, arm = session.tick(time.monotonic(), rospy.Time.now().to_nsec())
            publisher.publish(Bool(data=permitted))
            if arm:
                try:
                    result = gateway(True)
                    session.arm_result(result.success)
                    print("Gateway: " + result.message, flush=True)
                except rospy.ServiceException:
                    session.stop("gateway_service_failed")
            if session.phase == "stopped" and previous_phase not in (None, "stopped"):
                disarm()
            if session.phase != previous_phase:
                print("%s: %s" % (session.phase, session.reason), flush=True)
                previous_phase = session.phase
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        session.stop("console_closed")
        # rospy may already be shutting down; publisher can then reject writes.
        try:
            disarm()
        except rospy.ROSException:
            pass


if __name__ == "__main__":
    main()
