#!/usr/bin/env python3
"""
RoboClaw & Pixhawk Interface Node for Henes Broon T870 Autonomous Vehicle
- Steering: RoboClaw Solo 30A via USB Packet Serial (Closed-loop analog potentiometer feedback)
- Throttle: Cytron MDD20A via Pixhawk RC PWM (Channel 3 override / setpoint)
- Emergency Stop: 2D LaserScan (/scan) based sudden obstacle / child dummy detection
- GPS/Odometry: Subscribes to Pixhawk-fused RTK Odometry and NavSatFix topics
"""

import struct
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, LaserScan
from std_msgs.msg import Float32, Bool
from mavros_msgs.msg import OverrideRCIn

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class RoboClawSerial:
    """RoboClaw Packet Serial Interface with CRC16 calculation"""
    def __init__(self, port='COM3', baudrate=38400, address=0x80, logger=None):
        self.port = port
        self.baudrate = baudrate
        self.address = address
        self.logger = logger
        self.ser = None

        if SERIAL_AVAILABLE:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
                if self.logger:
                    self.logger.info(f"RoboClaw connected on {self.port} at {self.baudrate} baud")
            except Exception as e:
                if self.logger:
                    self.logger.warn(f"Failed to open RoboClaw serial port {self.port}: {e}")

    def _crc16(self, packet: bytes) -> int:
        crc = 0
        for b in packet:
            crc = crc ^ (b << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    def send_command(self, cmd: int, data: bytes = b'') -> bool:
        if not self.ser or not self.ser.is_open:
            return False
        payload = bytes([self.address, cmd]) + data
        crc = self._crc16(payload)
        packet = payload + struct.pack('>H', crc)
        try:
            self.ser.write(packet)
            self.ser.flush()
            return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error sending serial command {cmd}: {e}")
            return False

    def drive_position(self, position: int):
        """Command 65: Drive M1 to position (Analog feedback 0-1023)"""
        position = max(0, min(1023, int(position)))
        data = struct.pack('>I', position) + b'\x01'
        return self.send_command(65, data)

    def set_duty_m1(self, duty: int):
        """Command 32: Drive M1 Duty Cycle (-1500 to +1500)"""
        duty = max(-1500, min(1500, int(duty)))
        data = struct.pack('>h', duty)
        return self.send_command(32, data)


class T870ControlNode(Node):
    """
    ROS 2 Node managing T870 Steering, Throttle, and 2D LaserScan E-Stop
    """
    def __init__(self):
        super().__init__('roboclaw_steering_node')

        # Parameters
        self.declare_parameter('serial_port', 'COM3')
        self.declare_parameter('baudrate', 38400)
        self.declare_parameter('roboclaw_address', 128)
        self.declare_parameter('max_steer_angle_rad', 0.5236)  # ~30 degrees
        self.declare_parameter('pot_center', 512)
        self.declare_parameter('pot_min_limit', 212)   # Safe soft min
        self.declare_parameter('pot_max_limit', 812)   # Safe soft max
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('scan_timeout_sec', 0.5)

        # E-stop Parameters
        self.declare_parameter('estop_distance_m', 1.2)        # Emergency stop threshold
        self.declare_parameter('estop_angle_deg', 30.0)        # Front fan angle +/- deg

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value
        addr = self.get_parameter('roboclaw_address').value

        self.max_steer_rad = self.get_parameter('max_steer_angle_rad').value
        self.pot_center = self.get_parameter('pot_center').value
        self.pot_min = self.get_parameter('pot_min_limit').value
        self.pot_max = self.get_parameter('pot_max_limit').value

        self.estop_dist = self.get_parameter('estop_distance_m').value
        self.estop_angle_rad = math.radians(self.get_parameter('estop_angle_deg').value)
        self.command_timeout = float(self.get_parameter('command_timeout_sec').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout_sec').value)

        self.is_estop = True
        self.obstacle_estop = False
        self.last_cmd_ns = 0
        self.last_scan_ns = 0

        # RoboClaw driver initialization
        self.roboclaw = RoboClawSerial(port=port, baudrate=baud, address=addr, logger=self.get_logger())

        # Subscribers
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.sub_pixhawk_odom = self.create_subscription(
            Odometry, '/mavros/global_position/local', self.pixhawk_odom_callback, 10)
        self.sub_pixhawk_gps = self.create_subscription(
            NavSatFix, '/mavros/global_position/raw/fix', self.pixhawk_gps_callback, 10)

        # Publishers
        self.pub_rc_override = self.create_publisher(
            OverrideRCIn, '/mavros/rc/override', 10)

        self.pub_steer_feedback = self.create_publisher(Float32, '/t870/steering_angle', 10)
        self.pub_speed_feedback = self.create_publisher(Float32, '/t870/current_speed', 10)
        self.pub_estop_status = self.create_publisher(Bool, '/t870/emergency_stop', 10)

        self.current_speed = 0.0
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_callback)
        self.get_logger().info("T870 RoboClaw Steering, Throttle & 2D E-Stop Node Initialized.")

    def scan_callback(self, scan_msg: LaserScan):
        """2D LaserScan Emergency Stop detection for sudden obstacles (child dummy)"""
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment

        estop_triggered = False

        for i, r in enumerate(scan_msg.ranges):
            if math.isnan(r) or math.isinf(r) or r <= 0.0:
                continue

            angle = angle_min + (i * angle_increment)
            # Normalize angle to -pi..pi
            angle = math.atan2(math.sin(angle), math.cos(angle))

            # Check if point is in front fan angle range
            if abs(angle) <= self.estop_angle_rad:
                if r < self.estop_dist:
                    estop_triggered = True
                    break

        self.last_scan_ns = self.get_clock().now().nanoseconds
        if estop_triggered != self.obstacle_estop:
            self.obstacle_estop = estop_triggered
            if self.obstacle_estop:
                self.get_logger().warn(f"🚨 EMERGENCY STOP TRIGGERED! Sudden obstacle detected within {self.estop_dist}m.")
            else:
                self.get_logger().info("✅ Emergency stop cleared. Resuming normal operation.")

        self.update_estop_state()

    def is_fresh(self, stamp_ns, timeout):
        return stamp_ns and (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9 <= timeout

    def update_estop_state(self):
        self.is_estop = (self.obstacle_estop
                         or not self.is_fresh(self.last_scan_ns, self.scan_timeout)
                         or not self.is_fresh(self.last_cmd_ns, self.command_timeout))
        estop_msg = Bool()
        estop_msg.data = self.is_estop
        self.pub_estop_status.publish(estop_msg)

    def force_stop(self):
        self.send_pixhawk_throttle(0.0)
        self.roboclaw.drive_position(self.pot_center)

    def watchdog_callback(self):
        self.update_estop_state()
        if self.is_estop:
            self.force_stop()

    def steer_angle_to_pot(self, steer_rad: float) -> int:
        clamped_rad = max(-self.max_steer_rad, min(self.max_steer_rad, steer_rad))
        norm = clamped_rad / self.max_steer_rad
        pot_range = (self.pot_max - self.pot_min) / 2.0
        pot_target = int(self.pot_center + (norm * pot_range))
        return max(self.pot_min, min(self.pot_max, pot_target))

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_ns = self.get_clock().now().nanoseconds
        self.update_estop_state()
        # If Emergency Stop active, override throttle to 0 and maintain center steering
        if self.is_estop:
            self.force_stop()
            return

        # 1. Steering Control (RoboClaw Solo 30A)
        steer_rad = msg.angular.z
        pot_target = self.steer_angle_to_pot(steer_rad)
        self.roboclaw.drive_position(pot_target)

        fb_msg = Float32()
        fb_msg.data = float(steer_rad)
        self.pub_steer_feedback.publish(fb_msg)

        # 2. Throttle Control (Pixhawk RC Ch3 for Cytron MDD20A)
        linear_v = msg.linear.x
        self.send_pixhawk_throttle(linear_v)

    def send_pixhawk_throttle(self, linear_velocity: float):
        max_speed = 5.0
        norm_v = max(-1.0, min(1.0, linear_velocity / max_speed))
        pwm_val = int(1500 + (norm_v * 500))

        rc_msg = OverrideRCIn()
        rc_msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 8
        rc_msg.channels[2] = pwm_val
        self.pub_rc_override.publish(rc_msg)

    def pixhawk_odom_callback(self, msg: Odometry):
        self.current_speed = msg.twist.twist.linear.x
        spd_msg = Float32()
        spd_msg.data = self.current_speed
        self.pub_speed_feedback.publish(spd_msg)

    def pixhawk_gps_callback(self, msg: NavSatFix):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = T870ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.force_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
