#!/usr/bin/env python3
import rclpy
from .timed_parking import TimedParking
class ParallelPark(TimedParking):
    def __init__(self):super().__init__('parallel_park_node','PARALLEL','/cmd_vel/parallel','1.0,0.20,0.0;2.0,-0.20,0.25;2.0,-0.20,-0.25;0.5,0.0,0.0',-90.0)
def main(args=None):
    rclpy.init(args=args);n=ParallelPark()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.pub.publish(__import__('geometry_msgs.msg',fromlist=['Twist']).Twist());n.destroy_node();rclpy.shutdown() if rclpy.ok() else None
