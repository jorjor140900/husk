"""Keyboard teleop for camera assistance: i/w/up held by terminal key-repeat."""
import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class HeldForward:
    """Terminal keyboards have no key-up; expiry always bounds a held request."""
    def __init__(self,timeout=.6):
        if not math.isfinite(timeout) or not .1<=timeout<=.6:raise ValueError('key timeout must be 0.1..0.6 seconds')
        self.timeout=timeout;self.last=float('-inf')

    def key(self,key,now):
        self.last=now if key in ('i','w','\x1b[A') else float('-inf')

    def active(self,now):return 0<=now-self.last<self.timeout


def main(args=None):
    rclpy.init(args=args)
    node=Node('camera_assist_teleop')
    node.declare_parameter('speed',.18);node.declare_parameter('key_timeout',.6)
    speed=node.get_parameter('speed').value
    if not math.isfinite(speed) or not 0<speed<=.18:raise ValueError('speed must be within 0..0.18 m/s')
    held=HeldForward(node.get_parameter('key_timeout').value)
    output=node.create_publisher(Twist,'/husky_assist/teleop_cmd_vel',1)
    settings=None
    try:
        if not sys.stdin.isatty():raise RuntimeError('teleop needs an interactive terminal')
        settings=termios.tcgetattr(sys.stdin);tty.setraw(sys.stdin.fileno())
        print('Camera assist teleop: hold i/w/Up to move; k/Space/other key=STOP; Ctrl+C=quit.\r')
        print('Key release expires within 0.6s; hardware/operator STOP remains available.\r')
        while rclpy.ok():
            if select.select([sys.stdin],[],[],.05)[0]:
                key=os.read(sys.stdin.fileno(),1).decode("ascii",errors="ignore")
                if key=='\x03':break
                if key=='\x1b':
                    # Incomplete escape sequences stop, never latch forward.
                    for _ in range(2):
                        if select.select([sys.stdin],[],[],.01)[0]:key+=os.read(sys.stdin.fileno(),1).decode("ascii",errors="ignore")
                held.key(key,time.monotonic())
            command=Twist();command.linear.x=speed if held.active(time.monotonic()) else 0.
            output.publish(command)
            rclpy.spin_once(node,timeout_sec=0.)
    except KeyboardInterrupt:pass
    finally:
        if settings is not None:termios.tcsetattr(sys.stdin,termios.TCSADRAIN,settings)
        if rclpy.ok():output.publish(Twist())
        node.destroy_node();rclpy.try_shutdown()
