#!/usr/bin/env python3
"""Fail-safe Pure Pursuit tracker for the T870."""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from vehicle_msgs.msg import Waypoint, WaypointsArray

from .lib.utils_track import PIDController, PurePursuit


class T870TrackerNode(Node):
    def __init__(self):
        super().__init__('t870_tracker')
        defaults = {
            'target_velocity': 1.0,
            'vehicle_length': 1.08,
            'minimum_lookahead': 1.2,
            'lookahead_time': 0.8,
            'maximum_steering_angle': 0.5236,
            'input_timeout': 0.6,
            'use_speed_pid': True,
            # Pure Pursuit uses ROS +left internally; vehicle commands use +right.
            'positive_steering_is_right': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.target_vel = float(self.get_parameter('target_velocity').value)
        self.min_lookahead = float(self.get_parameter('minimum_lookahead').value)
        self.lookahead_time = float(self.get_parameter('lookahead_time').value)
        self.max_steer = float(self.get_parameter('maximum_steering_angle').value)
        self.input_timeout = float(self.get_parameter('input_timeout').value)
        self.use_speed_pid = bool(self.get_parameter('use_speed_pid').value)
        self.positive_steering_is_right = bool(
            self.get_parameter('positive_steering_is_right').value)
        vehicle_length = float(self.get_parameter('vehicle_length').value)

        self.pure_pursuit = PurePursuit(vehicle_length=vehicle_length)
        self.pid = PIDController(p=1.0, i=0.2, d=0.1, dt=0.05)
        self.current_speed = 0.0
        self.waypoints_msg = None
        self.last_odom_ns = 0
        self.last_waypoints_ns = 0
        self.was_stopped = True

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_target_point = self.create_publisher(
            Waypoint, '/t870/target_waypoint', 10)
        self.create_subscription(Odometry, '/mavros/global_position/local',
                                 self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(WaypointsArray, '/waypoints',
                                 self.waypoints_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('T870 fail-safe path tracker initialized at 20 Hz')

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x
        self.last_odom_ns = self.now_ns()

    def waypoints_callback(self, msg):
        self.waypoints_msg = msg
        self.last_waypoints_ns = self.now_ns()

    def fresh(self, stamp_ns):
        return stamp_ns and (self.now_ns() - stamp_ns) * 1e-9 <= self.input_timeout

    def publish_stop(self):
        self.pub_cmd_vel.publish(Twist())
        if not self.was_stopped:
            self.pid.reset()
        self.was_stopped = True

    def control_loop(self):
        if (not self.fresh(self.last_odom_ns)
                or not self.fresh(self.last_waypoints_ns)
                or self.waypoints_msg is None
                or not self.waypoints_msg.waypoints):
            self.publish_stop()
            return

        lookahead = max(self.min_lookahead,
                        abs(self.current_speed) * self.lookahead_time)
        steering, target = self.pure_pursuit.calculate_steering_angle(
            self.waypoints_msg, lookahead)
        if target is None or not math.isfinite(steering):
            self.publish_stop()
            return

        cmd = Twist()
        if self.use_speed_pid:
            throttle = self.pid.calculate(self.current_speed, self.target_vel)
            cmd.linear.x = max(0.0, min(self.target_vel * 1.5, throttle))
        else:
            # Arduino closes the wheel-speed PI loop. Indoor LiDAR mode uses a
            # fixed, tightly limited ROS speed instead of invalid GNSS odometry.
            cmd.linear.x = max(0.0, self.target_vel)
        command_steering = (-steering if self.positive_steering_is_right
                            else steering)
        cmd.angular.z = max(
            -self.max_steer, min(self.max_steer, float(command_steering)))
        self.pub_target_point.publish(target)
        self.pub_cmd_vel.publish(cmd)
        self.was_stopped = False


def main(args=None):
    rclpy.init(args=args)
    node = T870TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
