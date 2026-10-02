"""Safety-gated LiDAR-confirmed timed parking sequence base."""
import time
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from .mission_common import scan_points, twist

class TimedParking(Node):
    def __init__(self,node_name,mission,topic,default_steps,scan_center_deg):
        super().__init__(node_name);self.mission=mission;self.scan_center_deg=scan_center_deg
        for name,value in {'armed':False,'steps':default_steps,
            'maximum_speed_mps':0.35,'maximum_steer_rad':0.25,
            'require_scan_confirmation':True,'marker_distance_m':1.5,
            'marker_min_points':3,'scan_timeout_sec':0.5}.items():self.declare_parameter(name,value)
        self.pub=self.create_publisher(Twist,topic,10)
        self.active=False;self.started=0.;self.steps=[];self.confirmed=False;self.scan_t=0.
        self.create_subscription(String,'/t870/active_mission',self.on_mode,10)
        self.create_subscription(LaserScan,'/scan',self.on_scan,10)
        self.create_timer(.05,self.tick)
    def on_mode(self,msg):
        active=msg.data==self.mission
        if active and not self.active:
            self.started=0.;self.confirmed=False;self.steps=self.parse(str(self.get_parameter('steps').value))
            self.get_logger().warning(f'{self.mission} entered; armed={self.get_parameter("armed").value}')
        if self.active and not active:self.pub.publish(Twist())
        self.active=active
    def on_scan(self,msg):
        self.scan_t=time.monotonic()
        if not self.active or self.confirmed:return
        points=scan_points(msg,self.scan_center_deg,30.0,max_range=float(self.get_parameter('marker_distance_m').value))
        if len(points)>=int(self.get_parameter('marker_min_points').value):
            self.confirmed=True;self.started=time.monotonic()
            self.get_logger().warning(f'{self.mission} LiDAR marker confirmed; sequence started')
    @staticmethod
    def parse(text):
        return [tuple(float(v) for v in item.split(',')) for item in text.split(';') if item.strip()]
    def tick(self):
        if not self.active:return
        if not bool(self.get_parameter('armed').value):return self.pub.publish(Twist())
        if bool(self.get_parameter('require_scan_confirmation').value):
            if time.monotonic()-self.scan_t>float(self.get_parameter('scan_timeout_sec').value):return self.pub.publish(Twist())
            if not self.confirmed:return self.pub.publish(Twist())
        elif not self.started:self.started=time.monotonic()
        elapsed=time.monotonic()-self.started
        for duration,speed,steer in self.steps:
            if elapsed<=duration:
                vmax=float(self.get_parameter('maximum_speed_mps').value);smax=float(self.get_parameter('maximum_steer_rad').value)
                return self.pub.publish(twist(max(-vmax,min(vmax,speed)),max(-smax,min(smax,steer))))
            elapsed-=duration
        self.pub.publish(Twist())
