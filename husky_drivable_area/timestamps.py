"""ROS 2 timestamp conversion shared by assist and the command filter."""
def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec
