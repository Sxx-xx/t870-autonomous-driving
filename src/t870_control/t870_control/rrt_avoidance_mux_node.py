#!/usr/bin/env python3
"""Blend nominal lane/GPS commands with the local RRT avoidance command."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from vehicle_msgs.msg import WaypointsArray


def copy_twist(source):
    result = Twist()
    result.linear.x = float(source.linear.x)
    result.angular.z = float(source.angular.z)
    return result


class RrtAvoidanceMux(Node):
    """Use RRT only for a real detour, then smoothly return to nominal path."""

    def __init__(self):
        super().__init__('rrt_avoidance_mux')
        defaults = {
            'lane_input_topic': '/cmd_vel/lane',
            'gps_input_topic': '/cmd_vel/gps',
            'rrt_input_topic': '/cmd_vel/rrt',
            'lane_output_topic': '/cmd_vel/lane_safe',
            'gps_output_topic': '/cmd_vel/gps_safe',
            'waypoints_topic': '/waypoints',
            'command_timeout_sec': 0.75,
            'rrt_timeout_sec': 1.0,
            'detour_enter_lateral_m': 0.20,
            'detour_exit_lateral_m': 0.10,
            'detour_confirm_cycles': 2,
            'clear_confirm_cycles': 5,
            'blend_step': 0.12,
            'avoidance_speed_mps': 0.25,
            'maximum_steering_rad': 0.25,
            'output_rate_hz': 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        value = lambda name: self.get_parameter(name).value
        self.timeout = float(value('command_timeout_sec'))
        self.rrt_timeout = float(value('rrt_timeout_sec'))
        self.enter_lateral = float(value('detour_enter_lateral_m'))
        self.exit_lateral = float(value('detour_exit_lateral_m'))
        self.enter_cycles = int(value('detour_confirm_cycles'))
        self.exit_cycles = int(value('clear_confirm_cycles'))
        self.blend_step = float(value('blend_step'))
        self.avoidance_speed = float(value('avoidance_speed_mps'))
        self.max_steer = float(value('maximum_steering_rad'))
        rate = float(value('output_rate_hz'))
        if min(self.timeout, self.rrt_timeout, self.enter_lateral,
               self.blend_step, self.avoidance_speed, self.max_steer, rate) <= 0:
            raise ValueError('RRT avoidance parameters must be positive')

        self.lane = Twist()
        self.gps = Twist()
        self.rrt = Twist()
        self.lane_ns = self.gps_ns = self.rrt_ns = self.path_ns = 0
        self.lateral = 0.0
        self.enter_count = self.exit_count = 0
        self.detouring = False
        self.blend = 0.0

        self.lane_pub = self.create_publisher(
            Twist, str(value('lane_output_topic')), 10)
        self.gps_pub = self.create_publisher(
            Twist, str(value('gps_output_topic')), 10)
        self.active_pub = self.create_publisher(Bool, '/rrt/avoidance_active', 10)
        self.lateral_pub = self.create_publisher(
            Float32, '/rrt/maximum_lateral_offset', 10)
        self.create_subscription(
            Twist, str(value('lane_input_topic')), self.lane_callback, 10)
        self.create_subscription(
            Twist, str(value('gps_input_topic')), self.gps_callback, 10)
        self.create_subscription(
            Twist, str(value('rrt_input_topic')), self.rrt_callback, 10)
        self.create_subscription(
            WaypointsArray, str(value('waypoints_topic')), self.path_callback, 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            'RRT avoidance mux ready: lane/GPS nominal paths share local detours')

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def fresh(self, stamp, timeout=None):
        limit = self.timeout if timeout is None else timeout
        return bool(stamp) and (self.now_ns() - stamp) * 1e-9 <= limit

    def store(self, name, msg):
        if not (math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)):
            return
        setattr(self, name, copy_twist(msg))
        setattr(self, name + '_ns', self.now_ns())

    def lane_callback(self, msg):
        self.store('lane', msg)

    def gps_callback(self, msg):
        self.store('gps', msg)

    def rrt_callback(self, msg):
        self.store('rrt', msg)

    def path_callback(self, msg):
        self.path_ns = self.now_ns()
        self.lateral = max(
            (abs(float(point.y)) for point in msg.waypoints), default=0.0)
        if self.lateral >= self.enter_lateral:
            self.enter_count += 1
            self.exit_count = 0
            if self.enter_count >= self.enter_cycles:
                self.detouring = True
        elif self.lateral <= self.exit_lateral:
            self.exit_count += 1
            self.enter_count = 0
            if self.exit_count >= self.exit_cycles:
                self.detouring = False

    def safe_output(self, nominal, nominal_stamp):
        if not self.fresh(nominal_stamp):
            return Twist()
        if self.blend <= 0.0:
            return copy_twist(nominal)
        # A selected detour without fresh RRT steering must stop, never fall
        # through to a path known to contain an obstacle.
        if not (self.fresh(self.path_ns, self.rrt_timeout)
                and self.fresh(self.rrt_ns, self.rrt_timeout)):
            return Twist()
        output = Twist()
        rrt_speed = min(max(0.0, self.rrt.linear.x), self.avoidance_speed)
        nominal_speed = max(0.0, float(nominal.linear.x))
        output.linear.x = min(nominal_speed, rrt_speed)
        output.angular.z = (
            (1.0 - self.blend) * float(nominal.angular.z)
            + self.blend * float(self.rrt.angular.z))
        output.angular.z = max(-self.max_steer,
                               min(self.max_steer, output.angular.z))
        return output

    def tick(self):
        target_blend = 1.0 if self.detouring else 0.0
        if target_blend > self.blend:
            self.blend = min(target_blend, self.blend + self.blend_step)
        else:
            self.blend = max(target_blend, self.blend - self.blend_step)
        self.lane_pub.publish(self.safe_output(self.lane, self.lane_ns))
        self.gps_pub.publish(self.safe_output(self.gps, self.gps_ns))
        self.active_pub.publish(Bool(data=self.blend > 0.01))
        self.lateral_pub.publish(Float32(data=float(self.lateral)))


def main(args=None):
    rclpy.init(args=args)
    node = RrtAvoidanceMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.lane_pub.publish(Twist())
        node.gps_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
