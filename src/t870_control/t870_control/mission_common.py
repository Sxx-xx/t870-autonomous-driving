"""Shared helpers for fail-safe T870 mission nodes."""
import math
import time
from geometry_msgs.msg import Twist

MISSIONS = ('NONE', 'AVOID', 'ESTOP', 'PARALLEL', 'TPARK',
            'TRAFFIC_LIGHT', 'LANE_SIGNAL')

def twist(speed=0.0, steer=0.0):
    msg = Twist()
    msg.linear.x = float(speed)
    msg.angular.z = float(steer)
    return msg

def finite_twist(msg):
    return math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)

def scan_points(scan, center_deg, half_deg, min_range=0.05, max_range=10.0):
    points = []
    center = math.radians(center_deg)
    half = math.radians(half_deg)
    for i, value in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        error = math.atan2(math.sin(angle-center), math.cos(angle-center))
        if abs(error) <= half and math.isfinite(value):
            if max(scan.range_min, min_range) <= value <= min(scan.range_max, max_range):
                points.append((angle, float(value)))
    return points

def fresh(stamp, timeout):
    return stamp > 0.0 and time.monotonic() - stamp <= timeout
