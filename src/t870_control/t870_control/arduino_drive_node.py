#!/usr/bin/env python3
"""Fail-safe ROS 2 drive command bridge for the T870 Arduino controller."""

import math
import re
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from .control_utils import mps_to_kmh_command


class ArduinoDriveNode(Node):
    """Write signed km/h commands to Arduino and stop on stale safety inputs."""

    def __init__(self):
        super().__init__('arduino_drive_node')
        defaults = {
            'serial_port': '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_34331323136351211280-if00',
            'baud_rate': 115200,
            'max_speed_kmh': 2.0,
            'command_rate_hz': 10.0,
            'command_timeout_sec': 0.5,
            'estop_timeout_sec': 0.5,
            'reconnect_interval_sec': 1.0,
            # 대회 규정 정차에서 경사 밀림을 되받는다. /t870/competition_estop
            # 은 대회 launch 의 미션 매니저만 발행하므로 다른 launch 에는
            # 영향이 없다. 실차에서 문제가 생기면 false 로 두면 기존처럼
            # 즉시 PWM 차단으로 되돌아간다.
            'mission_hold_enabled': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.port = str(self.get_parameter('serial_port').value)
        self.baud = int(self.get_parameter('baud_rate').value)
        self.max_speed_kmh = float(self.get_parameter('max_speed_kmh').value)
        rate = float(self.get_parameter('command_rate_hz').value)
        self.command_timeout = float(self.get_parameter('command_timeout_sec').value)
        self.estop_timeout = float(self.get_parameter('estop_timeout_sec').value)
        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval_sec').value)
        self.mission_hold_enabled = bool(
            self.get_parameter('mission_hold_enabled').value)
        if min(rate, self.max_speed_kmh, self.command_timeout,
               self.estop_timeout, self.reconnect_interval) <= 0.0:
            raise ValueError('rates, limits, and timeouts must be positive')

        self.serial = None
        self.target_kmh = 0.0
        self.estop = True
        self.shift_estop = False
        self.mission_estop = False
        self.dynamic_estop = False
        self.manual_escape_allowed = False
        self.last_cmd_ns = 0
        self.last_estop_ns = 0
        self.last_shift_estop_ns = 0
        self.last_mission_estop_ns = 0
        self.last_dynamic_estop_ns = 0
        self.last_manual_escape_ns = 0
        self.last_connect_attempt = 0.0
        self.last_sent = None
        self.rx_buffer = ''
        self.rc_mode = 'LOST'

        self.pub_command = self.create_publisher(
            Float32, '/t870/arduino_drive_command_kmh', 10)
        self.pub_serial_connected = self.create_publisher(
            Bool, '/t870/arduino_serial_connected', 10)
        # 펌웨어는 Tgt(램프 목표)/PWM 도 보내는데 예전에는 Act 만 읽었다.
        # 그래서 '지령 8 km/h 인데 실제 1 km/h' 일 때 어디가 막힌 것인지
        # 가릴 수가 없었다. 셋을 같이 내보내면 바로 구분된다.
        #   Tgt 가 낮다        -> 명령이 펌웨어까지 안 온다 (ROS 쪽)
        #   Tgt 높고 PWM 낮다  -> PI 가 출력을 안 낸다 (제어)
        #   PWM 최대인데 느리다 -> 부하/전압 (기계)
        self.pub_drive_target = self.create_publisher(
            Float32, '/t870/drive_target_kmh', 10)
        self.pub_drive_pwm = self.create_publisher(
            Float32, '/t870/drive_pwm', 10)
        self.last_drive_log = 0.0
        self.pub_wheel_speed = self.create_publisher(
            Float32, '/wheel/speed_mps', 10)
        self.pub_wheel_odom = self.create_publisher(
            Odometry, '/wheel/odometry', 10)
        # Physical transmitter AUTO/MANUAL state, reported by the firmware
        # status line. This is the only path the RC mode switch has into ROS.
        self.pub_rc_mode = self.create_publisher(
            String, '/t870/remote_rc_mode', 10)
        self.pub_rc_auto = self.create_publisher(Bool, '/t870/remote_auto', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(
            Bool, '/t870/emergency_stop', self.estop_callback, 10)
        self.create_subscription(
            Bool, '/t870/shift_estop', self.shift_estop_callback, 10)
        self.create_subscription(
            Bool, '/t870/competition_estop', self.mission_estop_callback, 10)
        # 동적장애물. mission mux 가 명령을 0 으로 만들기는 하지만, 평범한
        # 0 명령은 펌웨어의 DECEL_RATE(0.7 km/h per sec) 램프를 탄다.
        # 8 km/h 에서 0 까지 11 초, 약 12 m 를 더 간다. 3 m 앞 장애물에는
        # 못 쓴다. 여기서 E 로 받아 PWM 을 즉시 끊는다.
        self.create_subscription(
            Bool, '/t870/mission_estop', self.dynamic_estop_callback, 10)
        self.create_subscription(
            Bool, '/t870/manual_escape_allowed',
            self.manual_escape_callback, 10)
        self.timer = self.create_timer(1.0 / rate, self.control_tick)
        self.pub_serial_connected.publish(Bool(data=False))
        self.get_logger().info(
            f'Arduino drive bridge ready: {self.port} at {self.baud} baud, '
            f'limit +/-{self.max_speed_kmh:.2f} km/h')

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def fresh(self, stamp_ns, timeout):
        return bool(stamp_ns) and (self.now_ns() - stamp_ns) * 1e-9 <= timeout

    def cmd_callback(self, msg):
        if not math.isfinite(msg.linear.x):
            self.get_logger().warning('Ignoring non-finite /cmd_vel.linear.x')
            return
        self.target_kmh = mps_to_kmh_command(
            msg.linear.x, self.max_speed_kmh)
        self.last_cmd_ns = self.now_ns()

    def estop_callback(self, msg):
        self.estop = bool(msg.data)
        self.last_estop_ns = self.now_ns()

    def shift_estop_callback(self, msg):
        self.shift_estop = bool(msg.data)
        self.last_shift_estop_ns = self.now_ns()

    def mission_estop_callback(self, msg):
        self.mission_estop = bool(msg.data)
        self.last_mission_estop_ns = self.now_ns()

    def dynamic_estop_callback(self, msg):
        self.dynamic_estop = bool(msg.data)
        self.last_dynamic_estop_ns = self.now_ns()

    def manual_escape_callback(self, msg):
        self.manual_escape_allowed = bool(msg.data)
        self.last_manual_escape_ns = self.now_ns()

    def safe_command(self):
        if not self.fresh(self.last_cmd_ns, self.command_timeout):
            return 0.0
        if not self.fresh(self.last_estop_ns, self.estop_timeout):
            return 0.0
        if self.estop:
            escape_fresh = self.fresh(
                self.last_manual_escape_ns, self.estop_timeout)
            # In MANUAL the operator has direct authority in both directions.
            # Command freshness and the Arduino watchdog remain mandatory.
            if escape_fresh and self.manual_escape_allowed:
                return self.target_kmh
            return 0.0
        if (self.fresh(self.last_shift_estop_ns, self.estop_timeout)
                and self.shift_estop):
            return 0.0
        if (self.fresh(self.last_mission_estop_ns, self.estop_timeout)
                and self.mission_estop):
            return 0.0
        if (self.fresh(self.last_dynamic_estop_ns, self.estop_timeout)
                and self.dynamic_estop):
            return 0.0
        return self.target_kmh

    def emergency_stop_required(self):
        """Request immediate PWM cut for stale safety data or an active E-stop."""
        if (self.fresh(self.last_shift_estop_ns, self.estop_timeout)
                and self.shift_estop):
            return True
        # 동적장애물은 홀드가 아니라 즉시 차단이다. 사람이 앞에 있다.
        if (self.fresh(self.last_dynamic_estop_ns, self.estop_timeout)
                and self.dynamic_estop):
            return True
        if (not self.mission_hold_enabled
                and self.fresh(self.last_mission_estop_ns, self.estop_timeout)
                and self.mission_estop):
            return True
        if not self.fresh(self.last_estop_ns, self.estop_timeout):
            return True
        if not self.estop:
            return False
        escape_fresh = self.fresh(
            self.last_manual_escape_ns, self.estop_timeout)
        return not (escape_fresh and self.manual_escape_allowed)

    def mission_hold_required(self):
        """Request an anti-rollback hold for a competition timed stop.

        경사에서 PWM 을 끊으면(coast) 그냥 뒤로 밀린다. 대회 규정 정차인
        WP39/WP532 는 '거기 서 있어야 하는' 정지이므로 구동을 놓지 않고
        밀림을 되받는다. 안전 E-stop 과 방향전환 E-stop 은 그대로 PWM 을
        끊으므로, 그쪽이 걸려 있으면 홀드를 요구하지 않는다.
        """
        if not self.mission_hold_enabled:
            return False
        if self.emergency_stop_required():
            return False
        return (self.fresh(self.last_mission_estop_ns, self.estop_timeout)
                and self.mission_estop)

    def connect(self):
        now = time.monotonic()
        if now - self.last_connect_attempt < self.reconnect_interval:
            return False
        self.last_connect_attempt = now
        try:
            self.serial = serial.Serial(
                self.port, self.baud, timeout=0, write_timeout=0.1)
            self.get_logger().info(f'Connected to Arduino on {self.port}')
            self.pub_serial_connected.publish(Bool(data=True))
            return True
        except (serial.SerialException, OSError) as error:
            self.serial = None
            self.get_logger().warning(f'Arduino connection failed: {error}')
            return False

    def disconnect(self, error=None):
        if error is not None:
            self.get_logger().error(f'Arduino serial error: {error}')
        if self.serial is not None:
            try:
                self.serial.close()
            except (serial.SerialException, OSError):
                pass
        self.serial = None
        self.last_sent = None
        self.rx_buffer = ''
        self.pub_serial_connected.publish(Bool(data=False))
        self.get_logger().warning(
            'Arduino serial disconnected; drive output is stopped')
        # Make the autonomous mode controller fail closed immediately. Without
        # this, it can keep GPS mode latched after Arduino telemetry disappears.
        if rclpy.ok():
            self.pub_rc_mode.publish(String(data='LOST'))
            self.pub_rc_auto.publish(Bool(data=False))

    def parse_telemetry(self, data):
        self.rx_buffer += data.decode('ascii', errors='ignore')
        lines = self.rx_buffer.split('\n')
        self.rx_buffer = lines.pop()
        for line in lines:
            rc_match = re.search(r'(?:^|\|\s*)RC:(AUTO|MANUAL|LOST)\b', line)
            if rc_match is not None:
                rc_mode = rc_match.group(1)
                self.rc_mode = rc_mode
                self.pub_rc_mode.publish(String(data=rc_mode))
                self.pub_rc_auto.publish(Bool(data=rc_mode == 'AUTO'))
            target = re.search(r'(?:^|\|\s*)Tgt:\s*(-?\d+(?:\.\d+)?)', line)
            pwm = re.search(r'(?:^|\|\s*)PWM:\s*(-?\d+(?:\.\d+)?)', line)
            if target is not None:
                self.pub_drive_target.publish(
                    Float32(data=float(target.group(1))))
            if pwm is not None:
                self.pub_drive_pwm.publish(Float32(data=float(pwm.group(1))))
            match = re.search(r'(?:^|\|\s*)Act:\s*(-?\d+(?:\.\d+)?)', line)
            if match is None:
                continue
            speed_mps = float(match.group(1)) / 3.6
            # 목표는 높은데 실제가 한참 낮으면 2 초마다 알린다. 원인을
            # 가릴 수 있도록 PWM 을 같이 찍는다.
            if target is not None and pwm is not None:
                want = float(target.group(1))
                got = float(match.group(1))
                now = time.monotonic()
                if (want >= 2.0 and got < want * 0.5
                        and now - self.last_drive_log >= 2.0):
                    self.last_drive_log = now
                    self.get_logger().warning(
                        '구동 부족: 지령 %.1f 목표 %.1f 실제 %.1f km/h, '
                        'PWM %s' % (self.target_kmh, want, got,
                                    pwm.group(1)))
            if not math.isfinite(speed_mps):
                continue
            stamp = self.get_clock().now().to_msg()
            self.pub_wheel_speed.publish(Float32(data=speed_mps))
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.covariance[0] = -1.0  # twist-only wheel measurement
            odom.twist.twist.linear.x = speed_mps
            odom.twist.covariance[0] = 0.04
            odom.twist.covariance[7] = 99999.0
            odom.twist.covariance[14] = 99999.0
            odom.twist.covariance[21] = 99999.0
            odom.twist.covariance[28] = 99999.0
            odom.twist.covariance[35] = 99999.0
            self.pub_wheel_odom.publish(odom)

    def write_command(self, speed_kmh, emergency_stop=False,
                      mission_hold=False):
        if self.serial is None and not self.connect():
            return
        try:
            waiting = self.serial.in_waiting
            if waiting:
                self.parse_telemetry(self.serial.read(waiting))
            # In physical MANUAL the Arduino firmware owns the throttle from
            # the RC receiver. Sending ROS 0/E commands here would repeatedly
            # override the stick command and cause stop-go behavior.
            if self.rc_mode == 'MANUAL':
                self.last_sent = None
                return
            if emergency_stop:
                # Firmware command 'E' bypasses its normal deceleration ramp,
                # clears the PI integral, and cuts drive PWM immediately.
                self.serial.write(b'E\n')
            elif mission_hold:
                # 'H' holds position against a slope instead of cutting PWM.
                # '0.000' 을 먼저 보내는 것은 구 펌웨어 안전장치다. 'H' 를
                # 모르는 펌웨어는 "? unknown cmd" 를 찍고 직전 목표 속도를
                # 그대로 유지하므로, 그것만 보내면 정차 자체가 안 된다.
                # 0 을 먼저 보내두면 최악의 경우에도 기존 동작(관성주행
                # 정지)으로 떨어진다.
                self.serial.write(b'0.000\nH\n')
            else:
                self.serial.write(f'{speed_kmh:.3f}\n'.encode('ascii'))
            self.last_sent = speed_kmh
        except (serial.SerialException, serial.SerialTimeoutException, OSError) as error:
            self.disconnect(error)

    def control_tick(self):
        command = self.safe_command()
        self.write_command(
            command,
            emergency_stop=self.emergency_stop_required(),
            mission_hold=self.mission_hold_required())
        self.pub_command.publish(Float32(data=command))

    def stop(self):
        if self.serial is not None:
            for _ in range(3):
                try:
                    self.serial.write(b'0.000\n')
                except (serial.SerialException, serial.SerialTimeoutException, OSError):
                    break
        self.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoDriveNode()
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
