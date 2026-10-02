#!/usr/bin/env python3
"""Confirm a lane signal and request GPS-path hot loading."""
import time,rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

class LaneSignal(Node):
    def __init__(self):
        super().__init__('lane_signal_node')
        for k,v in {'use_vision_detection':False,'confirm_frames':10,
                    'detection_timeout_sec':0.5,'lane_path_file':'',
                    'creep_speed_mps':0.20}.items():self.declare_parameter(k,v)
        self.active=False;self.label='';self.label_t=0.;self.count=0;self.sent=False
        self.pub=self.create_publisher(Twist,'/cmd_vel/lane_signal',10)
        self.path_pub=self.create_publisher(String,'/t870/set_gps_path',10)
        self.create_subscription(String,'/t870/active_mission',self.mode,10)
        self.create_subscription(String,'/vision/lane_signal',self.detect,10)
        self.create_timer(.05,self.tick)
    def mode(self,m):
        new=m.data=='LANE_SIGNAL'
        if new and not self.active:self.count=0;self.sent=False
        self.active=new
    def detect(self,m):self.label=m.data.strip().upper();self.label_t=time.monotonic()
    def tick(self):
        if not self.active:return
        enabled=bool(self.get_parameter('use_vision_detection').value)
        path=str(self.get_parameter('lane_path_file').value)
        valid=(enabled and bool(path) and
               time.monotonic()-self.label_t<=float(self.get_parameter('detection_timeout_sec').value)
               and self.label in ('DOWN','ARROW_DOWN','GO'))
        self.count=self.count+1 if valid else 0
        if self.count>=int(self.get_parameter('confirm_frames').value) and not self.sent:
            self.path_pub.publish(String(data=path));self.sent=True;self.get_logger().warning(f'GPS path switch requested: {path}')
        msg=Twist();msg.linear.x=float(self.get_parameter('creep_speed_mps').value) if valid else 0.0;self.pub.publish(msg)
def main(args=None):
    rclpy.init(args=args);n=LaneSignal()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.pub.publish(Twist());n.destroy_node();rclpy.shutdown() if rclpy.ok() else None
