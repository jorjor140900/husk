"""Ordered goals, with completion tied to the exact dispatched goal identity."""
import math


class WaypointQueue:
    def __init__(self):
        self.points=[];self.index=0;self.active=False;self.token=None

    def add(self,x,y):
        if len(self.points)>=128 or not all(math.isfinite(v) for v in (x,y)):
            raise ValueError('invalid_or_full_waypoint_queue')
        self.points.append((float(x),float(y)))

    def start(self):
        if self.active or not self.points:raise ValueError('queue_active_or_empty')
        self.index=0;self.active=True;self.token=None

    def reached(self,token):
        if not self.active or self.token is None or token!=self.token:return False
        self.index+=1;self.token=None
        if self.index==len(self.points):self.active=False
        return True

    def clear(self):
        self.__init__()
