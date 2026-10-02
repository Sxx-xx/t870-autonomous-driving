#!/usr/bin/env python3
"""Traffic-light mission using a detector label topic or a safe timed fallback."""
import time,rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

class TrafficLight(Node):
    def __init__(self):
        super().__init__('traffic_light_node')
        for k,v in {'use_vision_detection':False,'green_confirm_frames':10,
                    'detection_timeout_sec':0.5,'timed_stop_sec':3.5,
                    'proceed_speed_mps':0.25}.items():self.declare_parameter(k,v)
        self.active=False;self.entered=0.;self.label='UNKNOWN';self.label_t=0.;self.green_count=0
        self.pub=self.create_publisher(Twist,'/cmd_vel/traffic_light',10)
        self.create_subscription(String,'/t870/active_mission',self.mode,10)
        self.create_subscription(String,'/vision/traffic_light',self.detect,10)
        self.create_timer(.05,self.tick)
    def mode(self,m):
        new=m.data=='TRAFFIC_LIGHT'
        if new and not self.active:self.entered=time.monotonic();self.green_count=0
        self.active=new
    def detect(self,m):self.label=m.data.strip().upper();self.label_t=time.monotonic()
    def tick(self):
        if not self.active:return
        go=False
        if bool(self.get_parameter('use_vision_detection').value):
            fresh=time.monotonic()-self.label_t<=float(self.get_parameter('detection_timeout_sec').value)
            self.green_count=self.green_count+1 if fresh and self.label in ('GREEN','GO') else 0
            go=self.green_count>=int(self.get_parameter('green_confirm_frames').value)
        else:go=time.monotonic()-self.entered>=float(self.get_parameter('timed_stop_sec').value)
        msg=Twist();msg.linear.x=float(self.get_parameter('proceed_speed_mps').value) if go else 0.0
        self.pub.publish(msg)
def main(args=None):
    rclpy.init(args=args);n=TrafficLight()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.pub.publish(Twist());n.destroy_node();rclpy.shutdown() if rclpy.ok() else None
