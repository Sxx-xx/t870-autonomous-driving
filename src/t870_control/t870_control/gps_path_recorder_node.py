#!/usr/bin/env python3
"""Record ROS 2 NavSatFix samples as reusable T870 GPS paths."""

import csv
import math
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from pyproj import Transformer
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, UInt32


class GpsPathRecorder(Node):
    def __init__(self):
        super().__init__('gps_path_recorder')
        self.declare_parameter('fix_topic', '/gps/fix')
        self.declare_parameter(
            'output_directory',
            '/home/sxx/Desktop/colcon_ws./colcon_ws/colcon_ws/gps_recordings')
        self.declare_parameter('path_name', '')
        self.declare_parameter('minimum_distance_m', 1.0)
        self.declare_parameter('target_speed', 1.0)
        self.declare_parameter('require_valid_fix', True)
        # 2=RTK, 1=DGPS, 0=단독측위. 경로는 RTK 로만 딴다. 보정이 끊긴
        # 구간의 점은 미터 단위로 틀어져 있어서 한 번 섞여 들어가면 그
        # 경로로는 영영 못 맞춘다. 건너뛰면 그 자리에 공백이 남고,
        # path_to_qgis.py 가 '공백 있음' 으로 알려 준다.
        self.declare_parameter('minimum_fix_status', 0)
        self.declare_parameter('start_recording', False)
        self.declare_parameter('overwrite_existing', False)

        self.fix_topic = str(self.get_parameter('fix_topic').value)
        output_directory = Path(str(
            self.get_parameter('output_directory').value)).expanduser()
        requested_name = str(self.get_parameter('path_name').value).strip()
        self.minimum_distance = max(
            0.0, float(self.get_parameter('minimum_distance_m').value))
        self.target_speed = float(self.get_parameter('target_speed').value)
        self.require_valid_fix = bool(
            self.get_parameter('require_valid_fix').value)
        self.minimum_fix_status = int(
            self.get_parameter('minimum_fix_status').value)
        self.recording_enabled = bool(
            self.get_parameter('start_recording').value)
        self.overwrite_existing = bool(
            self.get_parameter('overwrite_existing').value)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path_name = requested_name or f't870_gps_path_{timestamp}'
        output_directory.mkdir(parents=True, exist_ok=True)
        self.csv_path = output_directory / f'{self.path_name}.csv'
        self.txt_path = output_directory / f'{self.path_name}.txt'
        self.csv_file = None
        self.txt_file = None
        self.csv_writer = None

        self.transformer = None
        self.utm_zone = ''
        self.last_easting = None
        self.last_northing = None
        self.count = 0
        self.path_message = RosPath()
        self.path_message.header.frame_id = 'utm'
        self.path_pub = self.create_publisher(
            RosPath, '/gps/recorded_path', 1)
        self.count_pub = self.create_publisher(
            UInt32, '/gps/recorded_point_count', 10)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.recording_pub = self.create_publisher(
            Bool, '/gps/recording_active', state_qos)
        self.create_subscription(
            Bool, '/gps/recording_enabled',
            self.recording_callback, state_qos)
        self.subscription = self.create_subscription(
            NavSatFix, self.fix_topic, self.fix_callback,
            qos_profile_sensor_data)
        self.get_logger().info(
            f'GPS path recorder ready: {self.fix_topic}, '
            f'minimum spacing {self.minimum_distance:.2f} m')
        self.recording_pub.publish(Bool(data=self.recording_enabled))
        if self.recording_enabled:
            self.open_output_files()

    def open_output_files(self):
        if self.csv_file is not None:
            return True
        if self.csv_path.exists() or self.txt_path.exists():
            if self.overwrite_existing:
                self.get_logger().warning(
                    f'Overwriting existing path: {self.path_name}')
                self.csv_path.unlink(missing_ok=True)
                self.txt_path.unlink(missing_ok=True)
            else:
                self.get_logger().error(
                    f'Refusing to overwrite existing path: {self.path_name}')
                self.recording_enabled = False
                self.recording_pub.publish(Bool(data=False))
                return False
        self.csv_file = self.csv_path.open('x', newline='', encoding='utf-8')
        self.txt_file = self.txt_path.open('x', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'index', 'ros_time_sec', 'latitude', 'longitude', 'altitude_m',
            'utm_easting_m', 'utm_northing_m', 'utm_zone', 'fix_status',
            'covariance_x_m2', 'covariance_y_m2', 'target_speed',
        ])
        self.csv_file.flush()
        self.get_logger().warning(f'GPS recording STARTED: {self.csv_path}')
        return True

    def recording_callback(self, msg):
        requested = bool(msg.data)
        if requested == self.recording_enabled:
            return
        if requested and not self.open_output_files():
            return
        self.recording_enabled = requested
        if not requested and self.csv_file is not None:
            self.csv_file.flush()
            self.txt_file.flush()
            self.get_logger().warning(
                f'GPS recording STOPPED at {self.count} points')
        self.recording_pub.publish(Bool(data=self.recording_enabled))

    def configure_utm(self, latitude, longitude):
        zone_number = int((longitude + 180.0) / 6.0) + 1
        zone_number = max(1, min(60, zone_number))
        epsg = (32600 if latitude >= 0.0 else 32700) + zone_number
        self.transformer = Transformer.from_crs(
            'EPSG:4326', f'EPSG:{epsg}', always_xy=True)
        zone_letter = 'N' if latitude >= 0.0 else 'S'
        self.utm_zone = f'{zone_number}{zone_letter}'
        self.get_logger().info(
            f'UTM projection selected: EPSG:{epsg} ({self.utm_zone})')

    def fix_callback(self, msg):
        if not self.recording_enabled:
            return
        if self.require_valid_fix and msg.status.status < self.minimum_fix_status:
            self.get_logger().warning(
                f'fix status {msg.status.status} below '
                f'{self.minimum_fix_status}; sample skipped',
                throttle_duration_sec=5.0)
            return
        if not all(math.isfinite(value) for value in
                   (msg.latitude, msg.longitude, msg.altitude)):
            return
        if not (-90.0 <= msg.latitude <= 90.0
                and -180.0 <= msg.longitude <= 180.0):
            return

        if self.transformer is None:
            self.configure_utm(msg.latitude, msg.longitude)
        easting, northing = self.transformer.transform(
            msg.longitude, msg.latitude)
        if self.last_easting is not None:
            distance = math.hypot(
                easting - self.last_easting, northing - self.last_northing)
            if distance < self.minimum_distance:
                return

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        covariance_x = msg.position_covariance[0]
        covariance_y = msg.position_covariance[4]
        self.csv_writer.writerow([
            self.count, f'{stamp_sec:.9f}',
            f'{msg.latitude:.10f}', f'{msg.longitude:.10f}',
            f'{msg.altitude:.3f}', f'{easting:.4f}', f'{northing:.4f}',
            self.utm_zone, msg.status.status,
            f'{covariance_x:.4f}', f'{covariance_y:.4f}',
            f'{self.target_speed:.3f}',
        ])
        # Legacy T870 path_reader expects whitespace-separated x, y, z.
        self.txt_file.write(
            f'{easting:.4f}\t{northing:.4f}\t{self.target_speed:.3f}\n')
        self.csv_file.flush()
        self.txt_file.flush()

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'utm'
        pose.pose.position.x = easting
        pose.pose.position.y = northing
        pose.pose.position.z = self.target_speed
        pose.pose.orientation.w = 1.0
        self.path_message.header.stamp = msg.header.stamp
        self.path_message.poses.append(pose)
        self.path_pub.publish(self.path_message)

        self.last_easting = easting
        self.last_northing = northing
        self.count += 1
        self.count_pub.publish(UInt32(data=self.count))
        if self.count == 1 or self.count % 10 == 0:
            self.get_logger().info(
                f'Recorded {self.count} points; latest '
                f'{easting:.3f}, {northing:.3f}')

    def destroy_node(self):
        if self.csv_file is not None and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
        if self.txt_file is not None and not self.txt_file.closed:
            self.txt_file.flush()
            self.txt_file.close()
        if rclpy.ok():
            self.get_logger().info(
                f'GPS path saved: {self.count} points, {self.txt_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsPathRecorder()
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
