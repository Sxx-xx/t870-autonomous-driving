#!/usr/bin/env python3
"""Select a mission from the current GPS waypoint index."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt32
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from .mission_common import MISSIONS

class MissionZoneManager(Node):
    def __init__(self):
        super().__init__('mission_zone_manager_node')
        self.declare_parameter('waypoint_topic', '/gps/current_waypoint')
        self.declare_parameter('output_topic', '/t870/active_mission')
        # Empty strings disable zones until field waypoint indexes are measured.
        for mission in MISSIONS[1:]:
            self.declare_parameter(mission.lower() + '_ranges', '')
        self.ranges = {m: self.parse_ranges(str(self.get_parameter(
            m.lower() + '_ranges').value)) for m in MISSIONS[1:]}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(
            String, str(self.get_parameter('output_topic').value), qos)
        self.create_subscription(UInt32, str(self.get_parameter(
            'waypoint_topic').value), self.on_index, 10)
        self.current = None
        self.publish('NONE')
        self.create_timer(0.5, lambda: self.publish(self.current or 'NONE'))

    @staticmethod
    def parse_ranges(text):
        result = []
        for item in filter(None, (x.strip() for x in text.split(','))):
            fields = item.replace('-', ':').split(':')
            if len(fields) != 2:
                raise ValueError(f'invalid mission range: {item}')
            lo, hi = int(fields[0]), int(fields[1])
            result.append((min(lo, hi), max(lo, hi)))
        return result

    def publish(self, mission):
        if mission != self.current:
            self.current = mission
            self.get_logger().warning(f'Active mission -> {mission}')
        self.pub.publish(String(data=mission))

    def on_index(self, msg):
        index = int(msg.data)
        selected = 'NONE'
        # Same precedence as the control mux.
        for mission in ('ESTOP', 'PARALLEL', 'TPARK', 'AVOID',
                        'TRAFFIC_LIGHT', 'LANE_SIGNAL'):
            if any(lo <= index <= hi for lo, hi in self.ranges[mission]):
                selected = mission
                break
        self.publish(selected)

def main(args=None):
    rclpy.init(args=args); node = MissionZoneManager()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown() if rclpy.ok() else None
