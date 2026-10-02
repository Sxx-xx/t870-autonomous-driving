#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from .timed_parking import TimedParking
class TPark(TimedParking):
    def __init__(self):super().__init__('t_park_node','TPARK','/cmd_vel/tpark','2.5,-0.20,0.25;1.0,-0.15,0.0;0.5,0.0,0.0;2.5,0.20,-0.25',90.0)
def main(args=None):
    rclpy.init(args=args);n=TPark()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.pub.publish(Twist());n.destroy_node();rclpy.shutdown() if rclpy.ok() else None
