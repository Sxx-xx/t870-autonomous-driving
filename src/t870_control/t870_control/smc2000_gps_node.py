#!/usr/bin/env python3
"""Minimal NMEA-0183 bridge for the SMC-2000 (u-blox F9P USB port)."""

import math
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial


def nmea_degrees(value: str, hemisphere: str) -> float:
    raw = float(value)
    degrees = math.floor(raw / 100.0)
    result = degrees + (raw - degrees * 100.0) / 60.0
    return -result if hemisphere in ('S', 'W') else result


def checksum_ok(sentence: str) -> bool:
    if not sentence.startswith('$') or '*' not in sentence:
        return False
    body, expected = sentence[1:].split('*', 1)
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    try:
        return checksum == int(expected[:2], 16)
    except ValueError:
        return False


# GGA fix quality -> NavSatStatus. ROS has no RTK-specific constant, so both
# RTK modes map to GBAS (the most trusted value available) and DGPS/SBAS map to
# SBAS. Consumers can then require a minimum trust level.
#   0 no fix, 1 standalone, 2 DGPS, 4 RTK fixed, 5 RTK float
GGA_QUALITY_STATUS = {
    0: NavSatStatus.STATUS_NO_FIX,
    1: NavSatStatus.STATUS_FIX,
    2: NavSatStatus.STATUS_SBAS_FIX,
    4: NavSatStatus.STATUS_GBAS_FIX,
    5: NavSatStatus.STATUS_GBAS_FIX,
    9: NavSatStatus.STATUS_SBAS_FIX,
}
# GGA 품질 4=RTK Fix, 5=RTK Float. 둘 다 보정이 붙은 상태다.
RTK_QUALITIES = (4, 5)
GGA_QUALITY_NAME = {
    0: 'no fix', 1: 'standalone', 2: 'DGPS', 4: 'RTK fixed', 5: 'RTK float',
}


class Smc2000GpsNode(Node):
    def __init__(self):
        super().__init__('smc2000_gps_node')
        self.declare_parameter(
            'device',
            '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('horizontal_error_scale', 1.5)
        # RTK 가 끊기면 프로세스를 끝낸다. 런치가 respawn 으로 즉시 다시
        # 띄우면서 시리얼을 닫았다 연다. 수신기 쪽 스트림이 꼬여서 보정이
        # 안 붙는 경우를 노린 것이다. 원인 치료가 아니라 재시도다.
        self.declare_parameter('rtk_restart_enabled', False)
        self.declare_parameter('rtk_restart_after_sec', 5.0)

        self.device = self.get_parameter('device').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.error_scale = float(self.get_parameter('horizontal_error_scale').value)
        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, '/gps/velocity', 10)
        self.serial_port = None
        self.stop_event = threading.Event()
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        self.last_quality = None
        self.rtk_restart_enabled = bool(
            self.get_parameter('rtk_restart_enabled').value)
        self.rtk_restart_after = float(
            self.get_parameter('rtk_restart_after_sec').value)
        self.last_rtk_time = time.monotonic()
        if self.rtk_restart_enabled:
            self.create_timer(1.0, self.rtk_watchdog)
            self.get_logger().warning(
                'RTK watchdog on: the node exits after '
                f'{self.rtk_restart_after:.1f} s without an RTK fix so launch '
                'can respawn it')
        self.get_logger().info(f'SMC-2000 GPS ready: {self.device} at {self.baudrate} baud')

    def rtk_watchdog(self):
        """RTK 가 오래 없으면 프로세스를 끝낸다. 런치가 다시 띄운다.

        destroy_node 로 곱게 끝내면 rclpy.spin 이 반환할 때까지 시간이
        걸리고 시리얼 리더 스레드가 남는다. 여기서는 포트를 닫고 바로
        나간다. 어차피 런치가 1초 뒤 새 프로세스를 띄운다.
        """
        idle = time.monotonic() - self.last_rtk_time
        if idle < self.rtk_restart_after:
            return
        self.get_logger().error(
            f'No RTK fix for {idle:.1f} s; restarting the GPS node')
        self.stop_event.set()
        try:
            if self.serial_port is not None:
                self.serial_port.close()
        except (OSError, serial.SerialException):
            pass
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)

    def open_serial(self):
        try:
            self.serial_port = serial.Serial(self.device, self.baudrate, timeout=1.0)
            self.get_logger().info(f'Connected to SMC-2000 on {self.device}')
        except (OSError, serial.SerialException) as exc:
            self.serial_port = None
            self.get_logger().warning(f'GPS connection failed: {exc}', throttle_duration_sec=5.0)

    def read_loop(self):
        while rclpy.ok() and not self.stop_event.is_set():
            if self.serial_port is None or not self.serial_port.is_open:
                self.open_serial()
                if self.serial_port is None:
                    self.stop_event.wait(1.0)
                    continue
            try:
                raw = self.serial_port.readline()
                start = raw.find(b'$')
                if start < 0:
                    continue
                sentence = raw[start:].decode('ascii', errors='ignore').strip()
                if checksum_ok(sentence):
                    self.handle_sentence(sentence)
            except (OSError, serial.SerialException) as exc:
                self.get_logger().warning(f'GPS disconnected: {exc}', throttle_duration_sec=5.0)
                try:
                    self.serial_port.close()
                except Exception:
                    pass
                self.serial_port = None

    def handle_sentence(self, sentence: str):
        fields = sentence.split('*', 1)[0].split(',')
        sentence_type = fields[0][-3:]
        if sentence_type == 'GGA':
            self.publish_gga(fields)
        elif sentence_type == 'RMC':
            self.publish_rmc_velocity(fields)
        elif sentence_type == 'VTG':
            self.publish_vtg_velocity(fields)

    def publish_gga(self, fields):
        if len(fields) < 10 or not fields[2] or not fields[4]:
            return
        try:
            quality = int(fields[6] or 0)
            latitude = nmea_degrees(fields[2], fields[3])
            longitude = nmea_degrees(fields[4], fields[5])
            altitude = float(fields[9] or 'nan')
            hdop = float(fields[8] or 'nan')
        except ValueError:
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        # Carry the GGA fix quality through instead of flattening every
        # solution to STATUS_FIX. An RTK solution and a 7 m DGPS solution are
        # not interchangeable: when the NTRIP correction stream drops, this
        # receiver falls back to DGPS and the reported position steps several
        # metres, which a follower that cannot tell them apart will chase.
        msg.status.status = GGA_QUALITY_STATUS.get(
            quality, NavSatStatus.STATUS_NO_FIX)
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude
        if math.isfinite(hdop):
            sigma = max(0.5, hdop * self.error_scale)
            msg.position_covariance[0] = sigma * sigma
            msg.position_covariance[4] = sigma * sigma
            msg.position_covariance[8] = (2.0 * sigma) ** 2
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        if quality in RTK_QUALITIES:
            self.last_rtk_time = time.monotonic()
        if quality != self.last_quality:
            previous = GGA_QUALITY_NAME.get(self.last_quality, self.last_quality)
            current = GGA_QUALITY_NAME.get(quality, quality)
            self.get_logger().warning(
                f'GPS fix quality {previous} -> {current}. A drop out of RTK '
                'steps the reported position by several metres.')
            self.last_quality = quality
        self.fix_pub.publish(msg)

    def publish_rmc_velocity(self, fields):
        if len(fields) < 9 or fields[2] != 'A':
            return
        try:
            speed = float(fields[7] or 0.0) * 0.514444
            course = math.radians(float(fields[8] or 0.0))
        except ValueError:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = speed * math.sin(course)  # East
        msg.twist.linear.y = speed * math.cos(course)  # North
        self.velocity_pub.publish(msg)

    def publish_vtg_velocity(self, fields):
        """VTG 로도 속도와 방위를 낸다.

        follower 의 heading 은 이 토픽에서 나온다. 그런데 이 수신기는
        RMC 를 0.02 Hz(50초에 한 번)로만 내보내고 GGA/VTG 만 4 Hz 로
        낸다. RMC 만 쓰면 gps_course_hold(3초)가 금방 만료되어 heading 이
        route_tangent 으로 떨어지고, 그 상태에서는 조향이 0 으로 묶여
        차가 경로를 벗어난 채 직진한다. 실제로 그 일이 났다.
        VTG 는 4 Hz 로 같은 정보를 준다.

        필드: 1 진북 기준 방위(도), 5 노트, 7 km/h, 9 모드.
        정지 중에는 방위가 빈 칸이라 그때는 내보내지 않는다.
        """
        if len(fields) < 8 or not fields[1]:
            return
        try:
            course = math.radians(float(fields[1]))
            speed = float(fields[7] or 0.0) / 3.6
        except ValueError:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = speed * math.sin(course)   # East
        msg.twist.linear.y = speed * math.cos(course)   # North
        self.velocity_pub.publish(msg)

    def destroy_node(self):
        self.stop_event.set()
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
        if self.reader.is_alive():
            try:
                self.reader.join(timeout=0.5)
            except KeyboardInterrupt:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Smc2000GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
