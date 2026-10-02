#!/usr/bin/env python3
"""Fail-safe T870 interface using Pixhawk/ArduRover without an STM32 ECU."""

import math

import rclpy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

from .control_utils import normalized_to_pwm


class PixhawkVehicleInterfaceNode(Node):
    """Send steering to Pixhawk and publish the shared vehicle E-stop."""

    def __init__(self):
        super().__init__('pixhawk_vehicle_interface_node')
        defaults = {
            'steering_rc_channel': 1,
            'steering_pwm_min': 1000,
            'steering_pwm_trim': 1500,
            'steering_pwm_max': 2000,
            'max_steer_angle_rad': 0.5236,
            # Project convention: +angular.z = right, -angular.z = left.
            'reverse_steering_output': False,
            'command_rate_hz': 50.0,
            'command_timeout_sec': 0.5,
            'scan_timeout_sec': 0.5,
            'mode_timeout_sec': 0.5,
            'estop_distance_m': 1.2,
            'estop_angle_deg': 30.0,
            'lidar_forward_angle_deg': 0.0,
            # A fast object entering this wider/longer forward corridor
            # latches a stop even if it has not reached the hard E-stop zone.
            'dynamic_stop_distance_m': 1.5,
            'dynamic_stop_angle_deg': 45.0,
            'dynamic_stop_hold_sec': 3.0,
            'dynamic_min_closing_speed_mps': 1.2,
            'dynamic_min_range_jump_m': 0.35,
            'dynamic_min_points': 3,
            'require_lidar': True,
            'vehicle_estop_enabled': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.steer_channel = int(self.get_parameter('steering_rc_channel').value) - 1
        self.steer_min = int(self.get_parameter('steering_pwm_min').value)
        self.steer_trim = int(self.get_parameter('steering_pwm_trim').value)
        self.steer_max = int(self.get_parameter('steering_pwm_max').value)
        self.max_steer = float(self.get_parameter('max_steer_angle_rad').value)
        self.reverse_steering = bool(
            self.get_parameter('reverse_steering_output').value)
        command_rate = float(self.get_parameter('command_rate_hz').value)
        self.command_timeout = float(self.get_parameter('command_timeout_sec').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout_sec').value)
        self.mode_timeout = float(self.get_parameter('mode_timeout_sec').value)
        self.estop_distance = float(self.get_parameter('estop_distance_m').value)
        self.estop_angle = math.radians(float(self.get_parameter('estop_angle_deg').value))
        self.lidar_forward_angle = math.radians(float(
            self.get_parameter('lidar_forward_angle_deg').value))
        self.dynamic_stop_distance = float(
            self.get_parameter('dynamic_stop_distance_m').value)
        self.dynamic_stop_angle = math.radians(float(
            self.get_parameter('dynamic_stop_angle_deg').value))
        self.dynamic_stop_hold = float(
            self.get_parameter('dynamic_stop_hold_sec').value)
        self.dynamic_min_closing_speed = float(
            self.get_parameter('dynamic_min_closing_speed_mps').value)
        self.dynamic_min_range_jump = float(
            self.get_parameter('dynamic_min_range_jump_m').value)
        self.dynamic_min_points = int(
            self.get_parameter('dynamic_min_points').value)
        self.require_lidar = bool(self.get_parameter('require_lidar').value)
        self.vehicle_estop_enabled = bool(
            self.get_parameter('vehicle_estop_enabled').value)

        if not 0 <= self.steer_channel < 18:
            raise ValueError('steering RC channel must be between 1 and 18')
        if command_rate <= 0.0 or self.max_steer <= 0.0:
            raise ValueError('rates and command limits must be positive')
        if (self.dynamic_stop_distance <= 0.0
                or self.dynamic_stop_hold < 0.0
                or self.dynamic_min_closing_speed < 0.0
                or self.dynamic_min_range_jump < 0.0
                or self.dynamic_min_points < 1):
            raise ValueError('dynamic obstacle stop parameters are invalid')

        self.target_steer = 0.0
        self.current_speed = 0.0
        self.last_cmd_ns = 0
        self.last_scan_ns = 0
        self.last_mode_ns = 0
        self.control_mode = 'STOP'
        self.obstacle_estop = False
        self.dynamic_stop_until_ns = 0
        self.previous_scan_ranges = None
        self.previous_scan_ns = 0
        self.last_estop = None

        self.pub_override = self.create_publisher(
            OverrideRCIn, '/mavros/rc/override', 10)
        self.pub_steer = self.create_publisher(Float32, '/t870/steering_angle', 10)
        self.pub_speed = self.create_publisher(Float32, '/t870/current_speed', 10)
        self.pub_estop = self.create_publisher(Bool, '/t870/emergency_stop', 10)
        self.pub_manual_escape = self.create_publisher(
            Bool, '/t870/manual_escape_allowed', 10)
        self.pub_dynamic_stop = self.create_publisher(
            Bool, '/t870/dynamic_obstacle_stop', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(
            String, '/t870/control_mode', self.mode_callback, 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/mavros/global_position/local', self.odom_callback,
            qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / command_rate, self.control_tick)
        self.get_logger().info(
            f'T870 Pixhawk steering ready: RC CH{self.steer_channel + 1}; '
            'drive output is released for Arduino serial control')

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def fresh(self, stamp_ns, timeout):
        return bool(stamp_ns) and (self.now_ns() - stamp_ns) * 1e-9 <= timeout

    def cmd_vel_callback(self, msg: Twist):
        if not (math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)):
            self.get_logger().warning('Ignoring non-finite /cmd_vel')
            return
        self.target_steer = max(-self.max_steer, min(self.max_steer, float(msg.angular.z)))
        self.last_cmd_ns = self.now_ns()

    def odom_callback(self, msg: Odometry):
        self.current_speed = float(msg.twist.twist.linear.x)
        self.pub_speed.publish(Float32(data=self.current_speed))

    def mode_callback(self, msg: String):
        self.control_mode = str(msg.data).upper()
        self.last_mode_ns = self.now_ns()

    def scan_callback(self, scan: LaserScan):
        if not self.require_lidar:
            # GPS parking uses LiDAR only for the T-path decision. Do not let
            # cone returns latch the vehicle E-stop or dynamic stop.
            self.obstacle_estop = False
            self.dynamic_stop_until_ns = 0
            self.previous_scan_ranges = tuple(scan.ranges)
            self.previous_scan_ns = self.now_ns()
            self.last_scan_ns = self.previous_scan_ns
            return
        detected = False
        now_ns = self.now_ns()
        dynamic_hits = 0
        elapsed = (
            (now_ns - self.previous_scan_ns) * 1e-9
            if self.previous_scan_ns else 0.0)
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance <= 0.0:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            angle_error = angle - self.lidar_forward_angle
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            if abs(angle_error) <= self.estop_angle and distance < self.estop_distance:
                detected = True
            if (abs(angle_error) <= self.dynamic_stop_angle
                    and distance < self.dynamic_stop_distance
                    and self.previous_scan_ranges is not None
                    and index < len(self.previous_scan_ranges)
                    and 0.02 <= elapsed <= self.scan_timeout):
                previous = self.previous_scan_ranges[index]
                if math.isfinite(previous) and previous > 0.0:
                    range_drop = previous - distance
                    closing_speed = range_drop / elapsed
                    if (range_drop >= self.dynamic_min_range_jump
                            and closing_speed >=
                            self.dynamic_min_closing_speed):
                        dynamic_hits += 1
                elif distance < self.dynamic_stop_distance * 0.75:
                    # A close return appearing where the previous beam had no
                    # return is also treated as a sudden intrusion.
                    dynamic_hits += 1

        if dynamic_hits >= self.dynamic_min_points:
            hold_ns = int(self.dynamic_stop_hold * 1e9)
            was_active = now_ns < self.dynamic_stop_until_ns
            self.dynamic_stop_until_ns = max(
                self.dynamic_stop_until_ns, now_ns + hold_ns)
            if not was_active:
                self.get_logger().warning(
                    f'Fast forward obstacle detected ({dynamic_hits} points); '
                    f'holding E-stop for {self.dynamic_stop_hold:.1f} s')
        self.obstacle_estop = detected
        self.previous_scan_ranges = tuple(scan.ranges)
        self.previous_scan_ns = now_ns
        self.last_scan_ns = now_ns

    def dynamic_stop_active(self):
        return self.now_ns() < self.dynamic_stop_until_ns

    def local_estop(self):
        if not self.vehicle_estop_enabled:
            return False
        command_stale = not self.fresh(self.last_cmd_ns, self.command_timeout)
        lidar_stale = self.require_lidar and not self.fresh(
            self.last_scan_ns, self.scan_timeout)
        return (
            command_stale
            or lidar_stale
            or (self.require_lidar and (
                self.obstacle_estop or self.dynamic_stop_active())))

    def manual_escape_allowed(self):
        # MANUAL deliberately bypasses LiDAR obstacle/dynamic E-stop for direct
        # operator control. Stale mode or command data can never bypass it.
        return (
            self.control_mode == 'MANUAL'
            and self.fresh(self.last_mode_ns, self.mode_timeout)
            and self.fresh(self.last_cmd_ns, self.command_timeout)
        )

    def publish_override(self, steering_pwm):
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        msg.channels[self.steer_channel] = int(steering_pwm)
        self.pub_override.publish(msg)

    def release_steering_override(self):
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        msg.channels[self.steer_channel] = OverrideRCIn.CHAN_RELEASE
        self.pub_override.publish(msg)

    def control_tick(self):
        estop = self.local_estop()
        manual_escape = estop and self.manual_escape_allowed()
        # STOP must never reuse the previous autonomous steering command.
        # When Arduino/RC telemetry is lost, the last left/right override
        # otherwise remains active and the vehicle can spin in place.
        if self.control_mode == 'STOP' and not estop:
            # Give the physical transmitter back its steering channel.
            self.release_steering_override()
            reported_steer = 0.0
            self.pub_steer.publish(Float32(data=reported_steer))
            self.pub_estop.publish(Bool(data=estop))
            self.pub_manual_escape.publish(Bool(data=manual_escape))
            self.pub_dynamic_stop.publish(Bool(data=self.dynamic_stop_active()))
            return
        if (estop and not manual_escape):
            steering_pwm = self.steer_trim
            reported_steer = 0.0
        else:
            normalized_steer = self.target_steer / self.max_steer
            if self.reverse_steering:
                normalized_steer = -normalized_steer
            steering_pwm = normalized_to_pwm(
                normalized_steer,
                self.steer_min, self.steer_trim, self.steer_max)
            reported_steer = self.target_steer
        self.publish_override(steering_pwm)
        self.pub_steer.publish(Float32(data=reported_steer))
        self.pub_estop.publish(Bool(data=estop))
        self.pub_manual_escape.publish(Bool(data=manual_escape))
        self.pub_dynamic_stop.publish(Bool(data=self.dynamic_stop_active()))
        if estop != self.last_estop:
            if estop:
                self.get_logger().warning('Vehicle E-stop ACTIVE')
            else:
                self.get_logger().info('Vehicle E-stop cleared')
            self.last_estop = estop

    def stop(self):
        if not rclpy.ok():
            return
        for _ in range(3):
            self.publish_override(self.steer_trim)


def main(args=None):
    rclpy.init(args=args)
    node = PixhawkVehicleInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
