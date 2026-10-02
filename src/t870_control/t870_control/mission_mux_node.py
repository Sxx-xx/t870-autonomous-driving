#!/usr/bin/env python3
"""Fail-safe priority multiplexer for all autonomous mission commands."""
import time
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String
from .mission_common import finite_twist, twist

class MissionMux(Node):
    PRIORITY = ('ESTOP','PARALLEL','TPARK','AVOID','TRAFFIC_LIGHT','LANE_SIGNAL')
    TOPICS = {'PARALLEL':'/cmd_vel/parallel', 'TPARK':'/cmd_vel/tpark',
              'AVOID':'/cmd_vel/avoid', 'TRAFFIC_LIGHT':'/cmd_vel/traffic_light',
              'LANE_SIGNAL':'/cmd_vel/lane_signal'}
    def __init__(self):
        super().__init__('mission_mux_node')
        defaults = {'normal_input_topic':'/cmd_vel/lane_safe',
                    'gps_normal_input_topic':'/cmd_vel/gps_safe',
                    'output_topic':'/cmd_vel/mission_auto',
                    'gps_output_topic':'/cmd_vel/mission_gps',
                    'command_timeout_sec':0.5, 'rate_hz':20.0}
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.timeout = float(self.get_parameter('command_timeout_sec').value)
        self.active = 'NONE'; self.estop = False
        self.commands = {}; self.stamps = {}; self.normal = Twist(); self.normal_stamp=0.0
        self.gps_normal=Twist();self.gps_normal_stamp=0.0
        self.pub = self.create_publisher(Twist, str(self.get_parameter('output_topic').value), 10)
        self.gps_pub=self.create_publisher(Twist,str(self.get_parameter('gps_output_topic').value),10)
        self.state_pub = self.create_publisher(String, '/t870/selected_mission', 10)
        self.create_subscription(String, '/t870/active_mission', self.on_active, 10)
        self.create_subscription(Bool, '/t870/mission_estop', self.on_estop, 10)
        self.create_subscription(Twist, str(self.get_parameter('normal_input_topic').value), self.on_normal, 10)
        self.create_subscription(Twist,str(self.get_parameter('gps_normal_input_topic').value),self.on_gps_normal,10)
        for mission, topic in self.TOPICS.items():
            self.create_subscription(Twist, topic, lambda msg, m=mission:self.on_command(m,msg), 10)
        self.create_timer(1.0/float(self.get_parameter('rate_hz').value), self.tick)

    def now(self): return time.monotonic()
    def on_active(self,msg): self.active = msg.data if msg.data else 'NONE'
    def on_estop(self,msg): self.estop = bool(msg.data)
    def on_normal(self,msg):
        if finite_twist(msg): self.normal, self.normal_stamp = msg, self.now()
    def on_gps_normal(self,msg):
        if finite_twist(msg):self.gps_normal,self.gps_normal_stamp=msg,self.now()
    def on_command(self,mission,msg):
        if finite_twist(msg): self.commands[mission], self.stamps[mission] = msg, self.now()
    def tick(self):
        # ESTOP is a monitoring zone. It takes control only after the
        # detector asserts the stop signal; otherwise normal driving continues.
        selected = 'ESTOP' if self.estop else (
            'NONE' if self.active == 'ESTOP' else self.active)
        output = twist();gps_output=twist()
        if selected == 'NONE':
            if self.now()-self.normal_stamp <= self.timeout: output=self.normal
            if self.now()-self.gps_normal_stamp<=self.timeout:gps_output=self.gps_normal
        elif selected != 'ESTOP' and selected in self.commands:
            if self.now()-self.stamps.get(selected,0.0) <= self.timeout: output=self.commands[selected];gps_output=self.commands[selected]
        self.pub.publish(output);self.gps_pub.publish(gps_output); self.state_pub.publish(String(data=selected))

def main(args=None):
    rclpy.init(args=args); node=MissionMux()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally: node.pub.publish(Twist());node.gps_pub.publish(Twist()); node.destroy_node(); rclpy.shutdown() if rclpy.ok() else None
