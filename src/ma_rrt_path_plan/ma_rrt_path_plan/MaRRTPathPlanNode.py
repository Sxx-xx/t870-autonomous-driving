#!/usr/bin/env python3
"""Local obstacle-avoidance planner for the T870 ROS 2 vehicle."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, LaserScan, NavSatFix, NavSatStatus
from vehicle_msgs.msg import Waypoint, WaypointsArray

try:
    from mavros_msgs.msg import EstimatorStatus, GPSRAW
except ImportError:
    EstimatorStatus = None
    GPSRAW = None

from .ma_rrt import RRT


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MaRRTPathPlanNode(Node):
    def __init__(self):
        super().__init__('ma_rrt_path_plan_node')

        defaults = {
            'odom_topic': '/mavros/global_position/local',
            'scan_topic': '/scan',
            'imu_topic': '/mavros/imu/data',
            'gps_fix_topic': '/mavros/global_position/raw/fix',
            'world_frame': 'map',
            'base_frame': 'base_link',
            'obstacle_max_range': 10.0,
            'obstacle_radius': 0.55,
            # Collapse dense C1 returns into grid cells before collision checks.
            'obstacle_voxel_size': 0.10,
            # Ignore returns inside the measured vehicle/LiDAR self-reflection radius.
            # Keep zero by default until the mounted sensor footprint is measured.
            'self_filter_radius': 0.0,
            'planning_distance': 8.0,
            'planning_rate_hz': 2.0,
            'sensor_timeout': 0.75,
            # Static, motor-off LiDAR/RRT validation without GNSS/EKF.
            # Never enable this mode for vehicle motion.
            'bench_test_mode': False,
            # Robot-centric LiDAR planning without GNSS/EKF, strictly low speed.
            'indoor_lidar_mode': False,
            'bench_heading_yaw': 0.0,
            'require_rtk_quality': True,
            'require_rtk_fixed': True,
            'rtk_max_horizontal_stddev': 0.15,
            'gps_raw_topic': '/mavros/gpsstatus/gps1/raw',
            'estimator_status_topic': '/mavros/estimator_status',
            # RPLIDAR C1 pose relative to base_link. Set these to measured values.
            'lidar_x': 0.0,
            'lidar_y': 0.0,
            'lidar_yaw': 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.gps_fix_topic = self.get_parameter('gps_fix_topic').value
        self.world_frame = self.get_parameter('world_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_range = float(self.get_parameter('obstacle_max_range').value)
        self.obstacle_radius = float(self.get_parameter('obstacle_radius').value)
        self.obstacle_voxel_size = float(
            self.get_parameter('obstacle_voxel_size').value)
        self.self_filter_radius = float(
            self.get_parameter('self_filter_radius').value)
        self.planning_distance = float(self.get_parameter('planning_distance').value)
        self.planning_rate_hz = float(
            self.get_parameter('planning_rate_hz').value)
        self.sensor_timeout = float(self.get_parameter('sensor_timeout').value)
        self.bench_test_mode = bool(self.get_parameter('bench_test_mode').value)
        self.indoor_lidar_mode = bool(
            self.get_parameter('indoor_lidar_mode').value)
        if self.bench_test_mode and self.indoor_lidar_mode:
            raise ValueError('bench_test_mode and indoor_lidar_mode are mutually exclusive')
        self.local_lidar_mode = self.bench_test_mode or self.indoor_lidar_mode
        self.bench_heading_yaw = float(
            self.get_parameter('bench_heading_yaw').value)
        self.require_rtk = bool(self.get_parameter('require_rtk_quality').value)
        self.require_rtk_fixed = bool(self.get_parameter('require_rtk_fixed').value)
        self.rtk_max_stddev = float(self.get_parameter('rtk_max_horizontal_stddev').value)
        self.lidar_x = float(self.get_parameter('lidar_x').value)
        self.lidar_y = float(self.get_parameter('lidar_y').value)
        self.lidar_yaw = float(self.get_parameter('lidar_yaw').value)

        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = self.bench_heading_yaw if self.local_lidar_mode else 0.0
        self.obstacle_points_2d = []
        self.last_odom_ns = 0
        self.last_scan_ns = 0
        self.last_imu_ns = 0
        self.last_gps_ns = 0
        self.last_gps_raw_ns = 0
        self.gps_quality_ok = False
        self.rtk_fixed = False
        self.last_estimator_ns = 0
        self.estimator_ok = False
        self.last_stop_reason = None

        self.create_subscription(Odometry, self.odom_topic, self.odometry_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(NavSatFix, self.gps_fix_topic, self.gps_callback,
                                 qos_profile_sensor_data)
        if GPSRAW is not None:
            self.create_subscription(
                GPSRAW, self.get_parameter('gps_raw_topic').value,
                self.gps_raw_callback, qos_profile_sensor_data)
            self.create_subscription(
                EstimatorStatus,
                self.get_parameter('estimator_status_topic').value,
                self.estimator_status_callback, 10)
        elif self.require_rtk_fixed:
            self.get_logger().error(
                'mavros_msgs unavailable: RTK-fixed gate will keep the vehicle stopped')

        self.waypoints_pub = self.create_publisher(WaypointsArray, '/waypoints', 10)
        self.path_pub = self.create_publisher(Path, '/rrt_path', 10)
        self.rrt = RRT()
        if self.planning_rate_hz <= 0.0 or self.obstacle_voxel_size < 0.0:
            raise ValueError('planning rate must be positive and voxel size non-negative')
        self.timer = self.create_timer(1.0 / self.planning_rate_hz, self.plan_loop)
        self.get_logger().info('T870 RPLIDAR C1 / RTK-aware local planner initialized')
        if self.bench_test_mode:
            self.get_logger().warning(
                'BENCH TEST MODE: navigation health gates bypassed; keep motors off')
        if self.indoor_lidar_mode:
            self.get_logger().warning(
                'INDOOR LIDAR MODE: GNSS/EKF gates bypassed; low-speed operation only')

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def odometry_callback(self, msg):
        if self.local_lidar_mode:
            return
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y
        self.car_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.last_odom_ns = self.now_ns()

    def imu_callback(self, msg):
        if self.local_lidar_mode:
            return
        # MAVROS publishes the Pixhawk EKF attitude, including the IST8310 compass.
        q = msg.orientation
        if abs(q.x) + abs(q.y) + abs(q.z) + abs(q.w) > 0.5:
            self.car_yaw = quaternion_to_yaw(q)
            self.last_imu_ns = self.now_ns()

    def gps_callback(self, msg):
        self.last_gps_ns = self.now_ns()
        valid_fix = msg.status.status >= NavSatStatus.STATUS_FIX
        covariance_known = msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN
        if covariance_known:
            horizontal_stddev = math.sqrt(max(
                0.0, msg.position_covariance[0], msg.position_covariance[4]))
            self.gps_quality_ok = valid_fix and horizontal_stddev <= self.rtk_max_stddev
        else:
            self.gps_quality_ok = valid_fix and not self.require_rtk

    def gps_raw_callback(self, msg):
        self.last_gps_raw_ns = self.now_ns()
        self.rtk_fixed = msg.fix_type == GPSRAW.GPS_FIX_TYPE_RTK_FIXED

    def estimator_status_callback(self, msg):
        """Validate the Pixhawk EKF attitude and GNSS-aided navigation output."""
        self.last_estimator_ns = self.now_ns()
        self.estimator_ok = (
            msg.attitude_status_flag
            and msg.velocity_horiz_status_flag
            and msg.pos_horiz_abs_status_flag
            and not msg.gps_glitch_status_flag
            and not msg.accel_error_status_flag)

    def scan_callback(self, scan):
        obs = []
        voxel_obs = {}
        # Bench paths are expressed directly in the laser frame. Keep raw scan
        # coordinates in that frame while bench_heading_yaw selects which laser
        # direction corresponds to the vehicle's physical forward direction.
        if self.local_lidar_mode:
            cy = 1.0
            sy = 0.0
        else:
            cy = math.cos(self.car_yaw)
            sy = math.sin(self.car_yaw)
        for i, distance in enumerate(scan.ranges):
            if (not math.isfinite(distance) or distance < max(0.1, scan.range_min)
                    or distance > min(self.max_range, scan.range_max)):
                continue
            if distance < self.self_filter_radius:
                continue
            angle = scan.angle_min + i * scan.angle_increment + self.lidar_yaw
            bx = self.lidar_x + distance * math.cos(angle)
            by = self.lidar_y + distance * math.sin(angle)
            world_x = self.car_x + bx * cy - by * sy
            world_y = self.car_y + bx * sy + by * cy
            if self.obstacle_voxel_size > 0.0:
                key = (round(world_x / self.obstacle_voxel_size),
                       round(world_y / self.obstacle_voxel_size))
                voxel_obs[key] = (world_x, world_y)
            else:
                obs.append((world_x, world_y))
        if self.obstacle_voxel_size > 0.0:
            obs = list(voxel_obs.values())
        self.obstacle_points_2d = obs
        self.last_scan_ns = self.now_ns()

    def data_is_fresh(self, stamp_ns):
        return stamp_ns and (self.now_ns() - stamp_ns) * 1e-9 <= self.sensor_timeout

    def stop(self, reason):
        now = self.get_clock().now().to_msg()
        msg = WaypointsArray()
        msg.header.stamp = now
        msg.header.frame_id = self.base_frame
        self.waypoints_pub.publish(msg)
        # Clear RViz immediately instead of leaving the last valid detour on
        # screen, which can be mistaken for the planner's current heading.
        empty_path = Path()
        empty_path.header.stamp = now
        empty_path.header.frame_id = self.world_frame
        self.path_pub.publish(empty_path)
        if reason != self.last_stop_reason:
            self.get_logger().warn('Planner holding stop: ' + reason)
            self.last_stop_reason = reason

    def plan_loop(self):
        if not self.local_lidar_mode:
            if not self.data_is_fresh(self.last_odom_ns):
                return self.stop('odometry missing or stale')
            if not self.data_is_fresh(self.last_imu_ns):
                return self.stop('Pixhawk/IST8310 attitude missing or stale')
            if (not self.data_is_fresh(self.last_estimator_ns)
                    or not self.estimator_ok):
                return self.stop('Pixhawk EKF attitude/GNSS aiding is not healthy')
        if not self.data_is_fresh(self.last_scan_ns):
            return self.stop('RPLIDAR scan missing or stale')
        if (not self.local_lidar_mode and self.require_rtk
                and (not self.data_is_fresh(self.last_gps_ns)
                     or not self.gps_quality_ok)):
            return self.stop('RTK GNSS fix missing or outside covariance limit')
        if not self.local_lidar_mode and self.require_rtk_fixed and (
                not self.data_is_fresh(self.last_gps_raw_ns) or not self.rtk_fixed):
            return self.stop('F9P is not reporting RTK FIXED')

        forward_x = math.cos(self.car_yaw)
        forward_y = math.sin(self.car_yaw)
        lateral_x = -forward_y
        lateral_y = forward_x
        goal = (self.car_x + self.planning_distance * forward_x,
                self.car_y + self.planning_distance * forward_y)
        start = (self.car_x, self.car_y)
        if self.obstacle_points_2d:
            points = self.rrt.plan(start, goal, self.obstacle_points_2d,
                                   obstacle_radius=self.obstacle_radius)
            # A local planner must not require the exact point straight ahead
            # to remain free: an obstacle near that fixed endpoint otherwise
            # produces an empty path even though there is room on either side.
            # Only search alternate forward endpoints when the nominal goal is
            # unreachable, then select the shortest collision-free detour.
            if not points and self.local_lidar_mode:
                # Search the full planning horizon first. In a cluttered room
                # every endpoint at 2 m can itself be occupied even though a
                # safe shorter forward motion exists. Fall back one horizon
                # tier at a time; never publish a partial, unchecked RRT branch.
                distance_tiers = (
                    self.planning_distance,
                    self.planning_distance * 0.75,
                    self.planning_distance * 0.50,
                )
                for tier_index, forward_distance in enumerate(distance_tiers):
                    candidates = []
                    lateral_offsets = ((0.70, -0.70, 1.10, -1.10)
                                       if tier_index == 0
                                       else (0.0, 0.70, -0.70, 1.10, -1.10))
                    tier_goal = (
                        self.car_x + forward_distance * forward_x,
                        self.car_y + forward_distance * forward_y)
                    for lateral_offset in lateral_offsets:
                        alternate_goal = (
                            tier_goal[0] + lateral_offset * lateral_x,
                            tier_goal[1] + lateral_offset * lateral_y)
                        candidate = self.rrt.plan(
                            start, alternate_goal, self.obstacle_points_2d,
                            obstacle_radius=self.obstacle_radius)
                        if candidate:
                            length = sum(
                                math.hypot(x2 - x1, y2 - y1)
                                for (x1, y1), (x2, y2)
                                in zip(candidate, candidate[1:]))
                            candidates.append((length, candidate))
                    if candidates:
                        points = min(candidates, key=lambda item: item[0])[1]
                        break
        else:
            points = [start, goal]
        if not points or len(points) < 2:
            return self.stop('no collision-free path')

        self.publish_path(points)
        self.last_stop_reason = None

    def publish_path(self, points):
        now = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.frame_id = self.world_frame
        path_msg.header.stamp = now
        waypoint_msg = WaypointsArray()
        waypoint_msg.header.frame_id = self.base_frame
        waypoint_msg.header.stamp = now
        cy = math.cos(self.car_yaw)
        sy = math.sin(self.car_yaw)

        for x, y in points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            path_msg.poses.append(pose)

            dx = x - self.car_x
            dy = y - self.car_y
            local_x = cy * dx + sy * dy
            local_y = -sy * dx + cy * dy
            if local_x > 0.05:
                waypoint = Waypoint()
                waypoint.x = float(local_x)
                waypoint.y = float(local_y)
                waypoint_msg.waypoints.append(waypoint)

        if not waypoint_msg.waypoints:
            return self.stop('path contains no forward waypoint')
        self.path_pub.publish(path_msg)
        self.waypoints_pub.publish(waypoint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MaRRTPathPlanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop('planner shutdown')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
