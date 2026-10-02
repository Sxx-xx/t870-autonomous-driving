#!/usr/bin/env python3
"""Fail-safe UTM path follower for GPS routes recorded by the T870."""

import csv
import datetime
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped
from pyproj import Transformer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, String, UInt32


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def rate_limit_angle(previous, current, dt, maximum_rate):
    """Rate-limit a wrapped angle without introducing a +/-pi discontinuity."""
    if previous is None or dt <= 0.0:
        return wrap_angle(current)
    maximum_change = maximum_rate * dt
    change = max(
        -maximum_change,
        min(maximum_change, wrap_angle(current - previous)))
    return wrap_angle(previous + change)


def gps_update_is_plausible(previous, current, dt, base_jump, maximum_speed):
    """Reject a fix that implies motion well beyond the vehicle capability."""
    if previous is None:
        return True
    allowed_distance = base_jump + maximum_speed * max(0.0, dt)
    return math.hypot(
        current[0] - previous[0], current[1] - previous[1]
    ) <= allowed_distance


def directions_from_cusps(points, threshold_rad):
    """경로 모양만 보고 각 웨이포인트를 전진/후진으로 가른다.

    이웃한 두 구간의 방향이 threshold 이상 꺾이면 그 자리는 첨점이다.
    차가 서서 기어를 바꾸는 지점이지, 돌 수 있는 코너가 아니다. 첫
    구간을 전진으로 놓고 첨점을 만날 때마다 부호를 뒤집는다.

    T 자 주차처럼 걸어서 딴 경로는 사람이 그냥 이어 걸으므로 첨점이
    그대로 남는다. 그것만 보고 후진 구간을 자동으로 찾아낸다.

    반환값은 (점마다 True=후진/False=전진 목록, 첨점 색인 목록) 이다.
    """
    if len(points) < 3:
        return [False] * len(points), []
    reverse = [False] * len(points)
    cusps = []
    driving_reverse = False
    previous = math.atan2(points[1][1] - points[0][1],
                          points[1][0] - points[0][0])
    for index in range(1, len(points) - 1):
        bearing = math.atan2(points[index + 1][1] - points[index][1],
                             points[index + 1][0] - points[index][0])
        # 첨점 '위' 의 점은 아직 이전 방향이다. 차는 그 점까지 오던
        # 방향으로 도착해서 서고, 새 방향은 다음 점부터 쓴다. 그래야
        # 사람이 지도에서 읽는 번호와 맞는다 (첨점 13 -> 후진은 14 부터).
        reverse[index] = driving_reverse
        if abs(wrap_angle(bearing - previous)) >= threshold_rad:
            driving_reverse = not driving_reverse
            cusps.append(index)
        previous = bearing
    reverse[-1] = driving_reverse
    return reverse, cusps


def direction_run_bounds(direction_flags, index, reverse):
    """Return the contiguous direction run containing ``index``."""
    if not direction_flags:
        return None
    start = index
    while start > 0 and direction_flags[start - 1] == reverse:
        start -= 1
    end = index
    while (end < len(direction_flags) - 1
           and direction_flags[end + 1] == reverse):
        end += 1
    return (start, end)


def bounded_nearest_index(points, x, y, previous_index, initialized,
                          backward_window, forward_window, maximum_advance):
    """Find the nearest point locally and prevent a one-cycle index jump."""
    if not initialized:
        return min(
            range(len(points)),
            key=lambda index: math.hypot(
                points[index][0] - x, points[index][1] - y))
    start = max(0, previous_index - backward_window)
    stop = min(len(points), previous_index + forward_window + 1)
    candidate = min(
        range(start, stop),
        key=lambda index: math.hypot(
            points[index][0] - x, points[index][1] - y))
    return max(previous_index, min(candidate, previous_index + maximum_advance))


def speed_banded_lookahead(speed, speed_limits, lookaheads):
    """Select one fixed LAD per speed band; final value covers above 10 km/h."""
    for speed_limit, lookahead in zip(speed_limits, lookaheads):
        if speed <= speed_limit:
            return lookahead
    return lookaheads[-1]


def strip_comments(stream):
    """'#' 으로 시작하는 줄은 없는 것으로 친다.

    경로 일부를 지우지 않고 잠시 빼고 달려 보려고 쓴다. 점을 삭제하면
    되돌리기 어렵고 idx 가 밀려서 로그 비교가 안 된다. 주석은 '#' 만
    지우면 원래대로 돌아오고 idx 도 그대로다.
    """
    for line in stream:
        if line.lstrip().startswith('#'):
            continue
        yield line


def parse_path_switch_request(request):
    """Parse an atomic path/start/reverse request; plain paths remain valid."""
    fields = request.split('::')
    filename = fields[0]
    if not filename:
        raise ValueError('GPS path switch filename is empty')
    start_index = None
    reverse_ranges = None
    for field in fields[1:]:
        if field.startswith('start='):
            start_index = int(field[6:])
            if start_index < 0:
                raise ValueError(
                    'GPS path switch start index must be non-negative')
        elif field.startswith('reverse='):
            reverse_ranges = field[8:]
        else:
            raise ValueError(f'unknown GPS path switch option: {field}')
    return filename, start_index, reverse_ranges


def reverse_flags_from_ranges(point_count, text):
    flags = [False] * point_count
    cusps = []
    for item in filter(None, (part.strip() for part in text.split(','))):
        values = item.replace('-', ':').split(':')
        if len(values) != 2:
            raise ValueError(f'invalid reverse waypoint range: {item}')
        start, stop = int(values[0]), int(values[1])
        if start < 1 or stop < start or stop >= point_count:
            raise ValueError(
                f'reverse waypoint range {item} is outside 1..{point_count-1}')
        for index in range(start, stop + 1):
            flags[index] = True
        cusps.extend((start - 1, stop))
    return flags, cusps


def parse_waypoint_ranges(text):
    """'199:235, 300:310' 형태를 [(199,235),(300,310)] 로 바꾼다."""
    ranges = []
    for item in filter(None, (part.strip() for part in text.split(','))):
        values = item.replace('-', ':').split(':')
        if len(values) != 2:
            raise ValueError(f'invalid waypoint range: {item}')
        start, stop = int(values[0]), int(values[1])
        if stop < start:
            raise ValueError(f'waypoint range {item} is reversed')
        ranges.append((start, stop))
    return ranges


def lookahead_for_waypoint(waypoint, ranges, override, default):
    """그 WP 가 override 구간 안이면 짧은 LAD 를 쓴다.

    S자처럼 곡률이 큰 구간에서는 속도 밴드 LAD(2 km/h 에서 5 m)가 너무
    길어 순수추종이 코너를 크게 자른다. 실측(T1.csv WP199~235, 헤어핀
    반경 약 7 m)에서 최대 횡오차가
      LAD 5.0 m -> 0.80 m,  3.0 m -> 0.32 m,  1.5 m -> 0.25 m,  1.0 m -> 0.22 m
    였다. 조향률 제한(maximum_steering_rate_rad_s)이 있어 1.0 m 에서도
    진동하지 않는다.

    [주의] LAD 를 줄이면 경로로 되당기는 힘이 세진다. 같은 구간에서
    카메라 회피 보정을 쓰고 있다면 그만큼 덜 비키게 된다.
    """
    if override <= 0.0 or not ranges:
        return default
    for start, stop in ranges:
        if start <= waypoint <= stop:
            return override
    return default


def load_route(filename):
    path = Path(filename).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f'GPS route does not exist: {path}')
    points = []
    relative = False
    # target_speed 가 0 인 점은 '여기서 멈춘다' 는 뜻이다. 그런데 QGIS 는
    # 필드명을 10자로 잘라 내보내므로 대회용 CSV 는 target_speed 대신
    # target_spe 라는 이름을 갖는다. 그 경우 컬럼을 못 찾아 전 구간 속도가
    # 0 으로 읽히고, 결국 모든 웨이포인트가 정지점이 된다. 속도 컬럼이
    # 아예 없는 경로와 속도 0 을 명시한 경로를 구분해서 알려준다.
    has_speed = False
    if path.suffix.lower() == '.csv':
        with path.open(newline='', encoding='utf-8') as stream:
            reader = csv.DictReader(strip_comments(stream))
            fields = set(reader.fieldnames or ())
            has_speed = 'target_speed' in fields
            if {'utm_easting_m', 'utm_northing_m'} <= fields:
                for row in reader:
                    points.append((
                        float(row['utm_easting_m']),
                        float(row['utm_northing_m']),
                        float(row.get('target_speed', 0.0) or 0.0),
                    ))
            elif {'easting', 'northing'} <= fields:
                # Competition/QGIS exports use these UTM column names. Some
                # hand-inserted parking points intentionally have no lat/lon,
                # so prefer the complete UTM coordinates.
                for row in reader:
                    points.append((
                        float(row['easting']),
                        float(row['northing']),
                        float(row.get('target_speed', 0.0) or 0.0),
                    ))
            elif {'longitude', 'latitude'} <= fields:
                rows = list(reader)
                if not rows:
                    raise ValueError('longitude/latitude route is empty')
                first_longitude = float(rows[0]['longitude'])
                first_latitude = float(rows[0]['latitude'])
                zone = max(
                    1, min(60, int((first_longitude + 180.0) / 6.0) + 1))
                epsg = (32600 if first_latitude >= 0.0 else 32700) + zone
                transformer = Transformer.from_crs(
                    'EPSG:4326', f'EPSG:{epsg}', always_xy=True)
                for row in rows:
                    longitude = float(row['longitude'])
                    latitude = float(row['latitude'])
                    easting, northing = transformer.transform(
                        longitude, latitude)
                    points.append((
                        easting,
                        northing,
                        float(row.get('target_speed', 0.0) or 0.0),
                    ))
            elif {'distance', 'angle'} <= fields:
                # Relative route format used by tlqkfehoTek.csv.  Distance is
                # cumulative metres and angle is a compass bearing: north=0,
                # east=90, increasing clockwise.
                relative = True
                previous_distance = None
                previous_bearing = None
                east = 0.0
                north = 0.0
                for row in reader:
                    distance = float(row['distance'])
                    bearing = math.radians(float(row['angle']))
                    if previous_distance is not None:
                        step = distance - previous_distance
                        if step < 0.0:
                            raise ValueError(
                                'relative GPS route distance must increase')
                        # A row describes the heading starting at that
                        # chainage, so use the preceding row for this segment.
                        east += step * math.sin(previous_bearing)
                        north += step * math.cos(previous_bearing)
                    points.append((east, north, 0.0))
                    previous_distance = distance
                    previous_bearing = bearing
            else:
                raise ValueError(
                    'CSV route requires longitude/latitude, UTM columns, '
                    'or distance/angle columns')
    else:
        with path.open(encoding='utf-8') as stream:
            for line in stream:
                fields = line.split()
                if len(fields) < 2:
                    continue
                if len(fields) >= 3:
                    has_speed = True
                points.append((
                    float(fields[0]), float(fields[1]),
                    float(fields[2]) if len(fields) >= 3 else 0.0,
                ))
    if len(points) < 2:
        raise ValueError('GPS route must contain at least two valid points')
    return points, relative, has_speed


class GpsPathFollower(Node):
    def __init__(self):
        super().__init__('gps_path_follower')
        defaults = {
            'path_file': '',
            'fix_topic': '/gps/fix',
            'velocity_topic': '/gps/velocity',
            'imu_topic': '/mavros/imu/data',
            'output_topic': '/cmd_vel/gps',
            'tracking_log_enabled': True,
            'tracking_log_directory': (
                '/home/sxx/Desktop/colcon_ws./colcon_ws/colcon_ws/'
                'gps_recordings'),
            'control_rate_hz': 20.0,
            # Default GPS tracking speed: 10 km/h (also used when launched
            # directly without an overriding launch argument).
            'target_speed_mps': 2.7778,
            # 5 km/h GPS tracking: look farther ahead to avoid reacting to
            # centimetre-level position noise with rapid left/right steering.
            'minimum_lookahead_m': 2.5,
            'lookahead_speed_gain': 1.5,
            # Piecewise LAD tuning at 2 km/h intervals up to 10 km/h.
            'use_speed_banded_lookahead': True,
            # 특정 WP 구간에서만 LAD 를 덮어쓴다. S자/헤어핀처럼 곡률이
            # 큰 구간용이다. 빈 문자열이면 사용하지 않는다.
            'lookahead_override_ranges': '',
            'lookahead_override_m': 0.0,
            'lookahead_2kmh_m': 5.0,
            'lookahead_4kmh_m': 5.0,
            'lookahead_6kmh_m': 5.0,
            'lookahead_8kmh_m': 4.8,
            'lookahead_10kmh_m': 4.5,
            'wheelbase_m': 0.725,
            'maximum_steering_rad': 0.20,
            'goal_tolerance_m': 1.0,
            # Allow recovery from ordinary GPS wander at 10 km/h before
            # entering the fail-safe route-error stop.
            'maximum_route_error_m': 10.0,
            'sensor_timeout_sec': 1.0,
            # Minimum NavSatStatus to act on. 2 = GBAS, which smc2000_gps_node
            # publishes for an RTK solution. Anything less means the NTRIP
            # correction stream dropped and the position has stepped several
            # metres; driving on that is worse than holding still.
            'minimum_fix_status': 2,
            'gps_jump_base_tolerance_m': 3.0,
            'gps_jump_maximum_speed_mps': 6.0,
            # 점프를 거부할 때 self.position 을 갱신하지 않으므로, 한 번
            # 크게 튀면 다음 fix 도 똑같이 멀어 또 거부된다. 그대로 두면
            # 영원히 복구가 안 되고 follower 가 'GPS fix missing or stale'
            # 로 명령을 끊는다. 실차에서 45.90 m 점프가 5520 회 연속
            # 거부되면서 차가 간헐적으로 서고 속도가 안 올랐다.
            #
            # 이만큼 연속으로 거부되면 새 위치를 받아들이고 다시 잡는다.
            # 4 Hz 기준 20 회 = 5 초. 받아들인 위치가 정말 틀렸다면
            # maximum_route_error_m 가 따로 잡아 준다.
            # 0 이하면 복구하지 않는다(예전 동작).
            'gps_jump_recover_count': 20,
            # 경로의 target_speed 가 음수인 점은 후진 구간으로 본다.
            # T 자 주차처럼 "웨이포인트 A 에서 B 까지 후진" 을 경로 파일로
            # 표현하기 위한 것이다. 기본은 꺼 둔다.
            'allow_reverse': False,
            'reverse_waypoint_ranges': '',
            # 전/후진을 바꾸기 전에 이 속도 아래로 먼저 선다. 펌웨어도
            # REVERSE_GUARD_KMH(0.3) 로 같은 보호를 하지만, follower 가
            # 먼저 세워야 전환 지점이 예측 가능해진다.
            'reverse_switch_speed_mps': 0.15,
            # 전/후진 전환 직후 이 시간 동안 조향을 중립으로, 웨이포인트
            # 색인을 제자리에 묶는다. 펌웨어가 역토크로 세우는 동안이다.
            'shift_settle_sec': 1.2,
            # 후진 구간 끝점을 이만큼 지나면 더 가지 않고 그 자리에서
            # 주차로 친다. 위치가 조금 안 맞아도 멈추는 쪽이 낫다.
            'reverse_overshoot_m': 0.0,
            # 후진을 멈출 웨이포인트 번호. -1 이면 후진 구간의 끝점을
            # 쓴다. 경로가 되돌아 나오는 모양이면 구간 끝이 가장 깊은
            # 점이라, 그보다 앞에서 세우고 싶을 때 여기에 번호를 준다.
            'reverse_stop_waypoint': -1,
            'reverse_start_waypoint': -1,
            # target_speed 가 0 인 웨이포인트는 '여기서 멈춘다' 는 뜻이다.
            # 이 시간만큼 서 있다가 다음 점의 속도로 출발한다. T 자 주차에서
            # 칸 안쪽에 도달한 뒤 잠깐 서는 동작에 쓴다.
            'waypoint_hold_sec': 3.0,
            # 이웃 구간 사이 방향이 이만큼 꺾이면 첨점으로 보고 그 뒤를
            # 후진으로 돌린다. 경로에 음수 target_speed 를 손으로 넣지
            # 않아도 된다. 0 이면 이 판정을 쓰지 않는다.
            'reverse_cusp_deg': 45.0,
            # 다음 첨점이 이 거리 안에 들어오면 도착을 기다리지 않고 미리
            # 제동을 시작한다. 첨점에 닿은 뒤에야 세우려 하면 관성으로
            # 지나쳐 버린다. LAD 로 고른 목표점 하나만 보는 것이 아니라
            # 경로 앞쪽의 첨점을 따로 살핀다.
            'cusp_brake_distance_m': 1.5,
            # 후진 중에는 이 속도 아래의 GPS 방위를 믿지 않는다. 도플러
            # 방위는 저속에서 수십 도씩 흔들리는데, 주차 속도가 딱 그
            # 구간이다. 여기 걸리면 IMU(정렬된 것)나 경로 접선으로 넘어간다.
            'reverse_course_speed_mps': 0.6,
            # 후진 중 전방주시거리. 0 이면 LAD 를 쓰지 않고 바로 다음
            # 웨이포인트를 겨눈다. 주차 궤적은 짧고 촘촘해서 몇 m 앞을
            # 보면 중간 점들을 건너뛰고 코너 안쪽을 질러가 버린다.
            'reverse_lookahead_m': 0.0,
            # 후진 조향 부호를 전진 pure-pursuit 기준에서 뒤집는다.
            # 일부 주차 경로/차량 세팅은 현장 기준이 반대라 런치에서 끌 수
            # 있게 둔다.
            'invert_reverse_steering': True,
            'wheel_speed_topic': '/wheel/speed_mps',
            'nearest_search_backward_points': 5,
            'nearest_search_forward_points': 30,
            'maximum_waypoint_advance_points': 10,
            # GPS course needs motion to be meaningful, but 0.30 kept the
            # follower blind for 12 s on a standing start and 0.18 for about
            # 10 s. 0.10 hands over a real heading as soon as the vehicle is
            # perceptibly rolling, roughly halving the blind window.
            'minimum_course_speed_mps': 0.10,
            # Low-pass gain used to track the IMU-to-course offset.
            # The MAVROS yaw on this vehicle sat about 76 degrees off the
            # bearing GPS course reported, so at a standing start the follower
            # steered hard the wrong way. Left off until the compass is
            # trusted; GPS course alone covers every metre actually driven.
            # Hold the wheels straight for this long after arming. At a
            # standing start the controller lacks the one measurement it
            # needs: GPS course requires motion, and acting on a guessed
            # heading is what threw the vehicle off the path every time.
            'startup_straight_sec': 5.0,
            'use_imu_heading': False,
            'imu_align_gain': 0.02,
            # Steering ceiling while the heading is only the route tangent.
            # The tangent assumes the vehicle points along the path, and that
            # assumption measured 21 degrees wrong on a standing start: the
            # follower read alpha as -25 deg and pulled right when the true
            # heading put the target 1.8 deg away, i.e. straight ahead. Zero
            # means creep straight until GPS course supplies a real bearing.
            'route_tangent_max_steer_rad': 0.0,
            'gps_course_hold_sec': 3.0,
            'maximum_course_rate_rad_s': 1.0,
            'imu_yaw_offset_rad': 0.0,
            'positive_steering_is_right': True,
            'allow_route_heading_at_start': True,
            # Assume the vehicle is placed facing the first path segment.
            # This removes the old 2-3 m MANUAL course-learning requirement.
            'align_relative_route_heading_at_start': True,
            'learn_heading_from_gps_course': False,
            'require_relative_course_alignment': False,
            'steering_filter_alpha': 0.40,
            'steering_deadband_rad': 0.01,
            'maximum_steering_rate_rad_s': 0.60,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        path_file = str(self.get_parameter('path_file').value)
        if not path_file:
            raise ValueError('path_file is required for GPS path following')
        self.points, self.relative_route, self.route_has_speed = (
            load_route(path_file))
        self.fix_topic = str(self.get_parameter('fix_topic').value)
        self.velocity_topic = str(self.get_parameter('velocity_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.tracking_log_enabled = bool(
            self.get_parameter('tracking_log_enabled').value)
        self.tracking_log_directory = Path(str(
            self.get_parameter('tracking_log_directory').value)).expanduser()
        rate = float(self.get_parameter('control_rate_hz').value)
        self.target_speed = float(self.get_parameter('target_speed_mps').value)
        self.min_lookahead = float(
            self.get_parameter('minimum_lookahead_m').value)
        self.lookahead_gain = float(
            self.get_parameter('lookahead_speed_gain').value)
        self.use_speed_banded_lookahead = bool(self.get_parameter(
            'use_speed_banded_lookahead').value)
        self.lookahead_override = float(
            self.get_parameter('lookahead_override_m').value)
        self.lookahead_override_ranges = parse_waypoint_ranges(
            str(self.get_parameter('lookahead_override_ranges').value))
        if self.lookahead_override_ranges and self.lookahead_override > 0.0:
            self.get_logger().warning(
                'Lookahead override: %.2f m on WP %s'
                % (self.lookahead_override,
                   ', '.join('%d~%d' % r
                             for r in self.lookahead_override_ranges)))
        self.lookahead_speed_limits = tuple(
            kmh / 3.6 for kmh in (2.0, 4.0, 6.0, 8.0, 10.0))
        self.lookahead_by_speed = tuple(float(self.get_parameter(
            f'lookahead_{kmh}kmh_m').value)
            for kmh in (2, 4, 6, 8, 10))
        self.wheelbase = float(self.get_parameter('wheelbase_m').value)
        self.max_steer = float(
            self.get_parameter('maximum_steering_rad').value)
        self.goal_tolerance = float(
            self.get_parameter('goal_tolerance_m').value)
        self.max_route_error = float(
            self.get_parameter('maximum_route_error_m').value)
        self.sensor_timeout = float(
            self.get_parameter('sensor_timeout_sec').value)
        self.minimum_fix_status = int(
            self.get_parameter('minimum_fix_status').value)
        self.gps_jump_base_tolerance = float(self.get_parameter(
            'gps_jump_base_tolerance_m').value)
        self.gps_jump_maximum_speed = float(self.get_parameter(
            'gps_jump_maximum_speed_mps').value)
        self.gps_jump_recover_count = int(self.get_parameter(
            'gps_jump_recover_count').value)
        self.allow_reverse = bool(
            self.get_parameter('allow_reverse').value)
        self.reverse_waypoint_ranges = str(
            self.get_parameter('reverse_waypoint_ranges').value)
        self.reverse_switch_speed = float(
            self.get_parameter('reverse_switch_speed_mps').value)
        self.waypoint_hold = float(
            self.get_parameter('waypoint_hold_sec').value)
        self.shift_settle = float(
            self.get_parameter('shift_settle_sec').value)
        self.reverse_overshoot = float(
            self.get_parameter('reverse_overshoot_m').value)
        self.reverse_stop_waypoint = int(
            self.get_parameter('reverse_stop_waypoint').value)
        self.reverse_start_waypoint = int(
            self.get_parameter('reverse_start_waypoint').value)
        self.reverse_cusp = math.radians(
            float(self.get_parameter('reverse_cusp_deg').value))
        self.cusp_brake_distance = float(
            self.get_parameter('cusp_brake_distance_m').value)
        self.reverse_course_speed = float(
            self.get_parameter('reverse_course_speed_mps').value)
        self.reverse_lookahead = float(
            self.get_parameter('reverse_lookahead_m').value)
        self.invert_reverse_steering = bool(
            self.get_parameter('invert_reverse_steering').value)
        # 경로 첨점 판정은 위 파라미터가 다 읽힌 뒤에 해야 한다.
        self.cusp_indices = []
        self.cusp_reverse = self.build_cusp_reverse()
        self.wheel_speed_topic = str(
            self.get_parameter('wheel_speed_topic').value)
        self.nearest_search_backward = int(self.get_parameter(
            'nearest_search_backward_points').value)
        self.nearest_search_forward = int(self.get_parameter(
            'nearest_search_forward_points').value)
        self.maximum_waypoint_advance = int(self.get_parameter(
            'maximum_waypoint_advance_points').value)
        self.min_course_speed = float(
            self.get_parameter('minimum_course_speed_mps').value)
        self.startup_straight = float(
            self.get_parameter('startup_straight_sec').value)
        self.use_imu_heading = bool(
            self.get_parameter('use_imu_heading').value)
        self.imu_align_gain = float(self.get_parameter('imu_align_gain').value)
        self.route_tangent_max_steer = float(self.get_parameter(
            'route_tangent_max_steer_rad').value)
        self.gps_course_hold = float(
            self.get_parameter('gps_course_hold_sec').value)
        self.max_course_rate = float(
            self.get_parameter('maximum_course_rate_rad_s').value)
        self.imu_yaw_offset = float(
            self.get_parameter('imu_yaw_offset_rad').value)
        self.positive_right = bool(
            self.get_parameter('positive_steering_is_right').value)
        self.allow_route_heading = bool(
            self.get_parameter('allow_route_heading_at_start').value)
        self.align_relative_heading = bool(self.get_parameter(
            'align_relative_route_heading_at_start').value)
        self.learn_heading_from_course = bool(self.get_parameter(
            'learn_heading_from_gps_course').value)
        self.require_relative_course_alignment = bool(self.get_parameter(
            'require_relative_course_alignment').value)
        self.steering_filter_alpha = float(self.get_parameter(
            'steering_filter_alpha').value)
        self.steering_deadband = float(self.get_parameter(
            'steering_deadband_rad').value)
        self.max_steering_rate = float(self.get_parameter(
            'maximum_steering_rate_rad_s').value)
        if (rate <= 0.0 or self.target_speed < 0.0
                or self.min_lookahead <= 0.0 or self.wheelbase <= 0.0
                or self.max_steer <= 0.0 or self.sensor_timeout <= 0.0
                or min(self.lookahead_by_speed) <= 0.0
                or not 0.0 < self.steering_filter_alpha <= 1.0
                or self.steering_deadband < 0.0
                or self.max_steering_rate <= 0.0):
            raise ValueError('GPS path follower parameters are invalid')
        if self.gps_course_hold <= 0.0 or self.max_course_rate <= 0.0:
            raise ValueError('GPS course parameters are invalid')
        if (self.gps_jump_base_tolerance <= 0.0
                or self.gps_jump_maximum_speed <= 0.0
                or self.nearest_search_backward < 0
                or self.nearest_search_forward < 1
                or self.maximum_waypoint_advance < 1):
            raise ValueError('GPS position guard parameters are invalid')

        self.transformer = None
        self.position = None
        self.position_time = 0.0
        self.latitude = None
        self.longitude = None
        self.route_anchored = not self.relative_route
        self.fix_time = 0.0
        self.velocity = (0.0, 0.0)
        self.velocity_time = 0.0
        self.gps_course = None
        self.gps_course_time = 0.0
        self.imu_yaw = None
        self.imu_time = 0.0
        self.heading_offset = None
        self.nearest_index = 0
        self.last_logged_waypoint = None
        self.nearest_initialized = False
        self.control_mode = None
        self.fix_quality_ok = True
        self.armed_time = None
        self.startup_locked = True
        self.rejected_fix_count = 0
        self.gps_course_speed = 0.0
        self.driving_reverse = False
        self.reverse_start_consumed = False
        self.shift_pending_direction = None
        self.shift_time = -1e9
        # 후진 구간 끝 처리. 'brake' 는 세우는 중, 'hold' 는 정차 중.
        self.park_phase = None
        self.park_until = 0.0
        self.park_resume = 0
        # 지금 달리는 방향 구간의 색인 범위. 후진 중에 겹친 전진 구간으로
        # 색인이 넘어가면 목표점이 사라져 제어가 통째로 무너진다.
        self.active_run = None
        self.heading_reverse = False
        self.external_speed_limit = None
        self.cusp_indices = []
        self.hold_index = None
        self.shift_hold_index = None
        self.hold_until = 0.0
        # 엔코더 실차 속도. GPS 도플러 속도와 얼마나 어긋나는지 보려고
        # 기록만 한다. 제어에는 쓰지 않는다.
        self.wheel_speed = None
        self.wheel_speed_time = 0.0
        self.last_reason = None
        self.route_heading_warning_sent = False
        self.initial_heading_aligned = False
        self.relative_course_aligned = not self.relative_route
        self.relative_origin = None
        self.filtered_steering = 0.0
        self.last_control_time = time.monotonic()
        self.tracking_start_time = self.last_control_time
        self.tracking_log_stream = None
        self.tracking_log_writer = None
        self.tracking_log_path = None
        self.heading_source = 'none'

        if self.tracking_log_enabled:
            self.tracking_log_directory.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            self.tracking_log_path = (
                self.tracking_log_directory
                / f'gps_tracking_{timestamp}.csv')
            self.tracking_log_stream = self.tracking_log_path.open(
                'w', newline='', encoding='utf-8')
            fields = (
                'elapsed_sec', 'longitude', 'latitude',
                'utm_easting_m', 'utm_northing_m', 'speed_mps',
                'heading_rad', 'heading_source', 'waypoint_index', 'route_error_m',
                'lookahead_m', 'target_index', 'target_easting_m',
                'target_northing_m', 'alpha_rad', 'raw_steering_rad',
                'filtered_steering_rad', 'command_speed_mps',
                'wheel_speed_mps', 'speed_error_mps', 'state')
            self.tracking_log_writer = csv.DictWriter(
                self.tracking_log_stream, fieldnames=fields)
            self.tracking_log_writer.writeheader()
            self.tracking_log_stream.flush()

        self.command_pub = self.create_publisher(
            Twist, self.output_topic, 10)
        self.path_pub = self.create_publisher(
            RosPath, '/gps/reference_path', 1)
        self.index_pub = self.create_publisher(
            UInt32, '/gps/current_waypoint', 10)
        self.error_pub = self.create_publisher(
            Float32, '/gps/cross_track_error', 10)
        self.lookahead_pub = self.create_publisher(
            Float32, '/gps/lookahead_distance', 10)
        self.goal_pub = self.create_publisher(
            Bool, '/gps/goal_reached', 10)
        self.shift_estop_pub = self.create_publisher(
            Bool, '/t870/shift_estop', 10)
        self.create_subscription(
            NavSatFix, self.fix_topic, self.fix_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            Float32, self.wheel_speed_topic, self.wheel_speed_callback, 10)
        self.create_subscription(
            TwistStamped, self.velocity_topic, self.velocity_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            Imu, self.imu_topic, self.imu_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            String, '/t870/set_gps_path', self.set_path_callback, 10)
        self.create_subscription(
            String, '/t870/control_mode', self.control_mode_callback, 10)
        self.create_subscription(
            Float32, '/t870/gps_speed_limit', self.speed_limit_callback, 10)
        self.timer = self.create_timer(1.0 / rate, self.control_tick)
        self.publish_reference_path()
        self.get_logger().warning(
            f'GPS path follower ready with {len(self.points)} points; '
            f'output={self.output_topic}; '
            f'route={"relative" if self.relative_route else "UTM"}; '
            f'target_speed column={"yes" if self.route_has_speed else "no"} '
            f'(stop waypoints '
            f'{"enabled" if self.route_has_speed else "disabled"}); '
            'waiting for valid position/heading')
        if self.tracking_log_path is not None:
            self.get_logger().warning(
                f'GPS tracking CSV: {self.tracking_log_path}')

    def control_mode_callback(self, msg):
        # Re-entering GPS mode re-acquires the nearest waypoint over the whole
        # route. The bounded search is monotonic once initialized, so without
        # this the index stays wherever it locked on at launch and the vehicle
        # cannot follow the route after being repositioned in MANUAL.
        mode = str(msg.data).upper()
        if mode == self.control_mode:
            return
        previous = self.control_mode
        self.control_mode = mode
        if mode != 'GPS':
            self.command_pub.publish(Twist())
            self.shift_estop_pub.publish(Bool(data=False))
            self.shift_pending_direction = None
            self.last_reason = f'inactive control mode {mode}'
        if mode == 'GPS' and previous is not None:
            self.nearest_initialized = False
            self.last_logged_waypoint = None
            self.startup_locked = False
            self.armed_time = time.monotonic()
            self.get_logger().warning(
                f'Control mode {previous} -> GPS; driving straight for '
                f'{self.startup_straight:.1f} s, then locking on to the '
                f'nearest waypoint')

    def speed_limit_callback(self, msg):
        value = float(msg.data)
        if math.isfinite(value) and value > 0.0:
            self.external_speed_limit = value

    def set_path_callback(self, msg):
        """Atomically replace the route; invalid requests leave it unchanged."""
        try:
            filename, start_index, reverse_ranges = parse_path_switch_request(
                msg.data)
            points, relative, has_speed = load_route(filename)
            if start_index is not None and start_index >= len(points):
                raise ValueError(
                    f'GPS path switch start index {start_index} is outside '
                    f'0..{len(points) - 1}')
        except (OSError, ValueError) as error:
            self.get_logger().error(f'GPS path switch rejected: {error}')
            return
        self.points = points
        self.relative_route = relative
        self.route_has_speed = has_speed
        if reverse_ranges is not None:
            self.reverse_waypoint_ranges = reverse_ranges
        self.cusp_reverse = self.build_cusp_reverse()
        self.reverse_start_consumed = False
        self.route_anchored = not relative
        self.nearest_index = start_index or 0
        self.last_logged_waypoint = None
        # An explicit index is used for competition routes that overlap
        # themselves. Without it, a global nearest search can attach to the
        # wrong lap or mission branch at the same physical location.
        self.nearest_initialized = start_index is not None
        self.driving_reverse = False
        self.shift_pending_direction = None
        self.park_phase = None
        self.active_run = None
        self.hold_index = None
        self.shift_hold_index = None
        self.heading_offset = None
        self.initial_heading_aligned = False
        self.relative_course_aligned = not relative
        self.relative_origin = None
        self.filtered_steering = 0.0
        self.last_control_time = time.monotonic()
        self.publish_reference_path()
        self.get_logger().warning(
            f'GPS path hot-loaded: {filename} ({len(points)} points), '
            f'start={"nearest" if start_index is None else start_index}, '
            f'target_speed column={"yes" if has_speed else "no"}')

    def publish_reference_path(self):
        message = RosPath()
        message.header.frame_id = 'utm'
        message.header.stamp = self.get_clock().now().to_msg()
        for x, y, speed in self.points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = speed
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_pub.publish(message)

    def fix_callback(self, msg):
        if (not math.isfinite(msg.latitude)
                or not math.isfinite(msg.longitude)):
            return
        if msg.status.status < self.minimum_fix_status:
            if self.fix_quality_ok:
                self.fix_quality_ok = False
                self.get_logger().warning(
                    f'GPS fix status {msg.status.status} is below the required '
                    f'{self.minimum_fix_status}; ignoring these fixes. The '
                    'follower stops once the last good position goes stale.')
            return
        if not self.fix_quality_ok:
            self.fix_quality_ok = True
            self.get_logger().warning('GPS fix quality recovered')
        if self.transformer is None:
            zone = max(1, min(60, int((msg.longitude + 180.0) / 6.0) + 1))
            epsg = (32600 if msg.latitude >= 0.0 else 32700) + zone
            self.transformer = Transformer.from_crs(
                'EPSG:4326', f'EPSG:{epsg}', always_xy=True)
            self.get_logger().info(f'GPS follower using EPSG:{epsg}')
        candidate_position = self.transformer.transform(
            msg.longitude, msg.latitude)
        now = time.monotonic()
        dt = min(self.sensor_timeout, max(0.0, now - self.position_time))
        if not gps_update_is_plausible(
                self.position, candidate_position, dt,
                self.gps_jump_base_tolerance,
                self.gps_jump_maximum_speed):
            self.rejected_fix_count += 1
            jump = math.hypot(
                candidate_position[0] - self.position[0],
                candidate_position[1] - self.position[1])
            if (self.gps_jump_recover_count > 0
                    and self.rejected_fix_count >= self.gps_jump_recover_count):
                # 계속 거부만 하면 영영 못 돌아온다. 새 위치를 받아들이고
                # 다시 잡는다. 정말 틀린 위치라면 maximum_route_error_m 가
                # 곧바로 잡는다.
                self.get_logger().warning(
                    f'GPS jump {jump:.2f} m 이 {self.rejected_fix_count} 회 '
                    f'연속 거부됐다. 새 위치를 받아들이고 다시 잡는다')
                self.rejected_fix_count = 0
            else:
                if (self.rejected_fix_count == 1
                        or self.rejected_fix_count % 10 == 0):
                    self.get_logger().warning(
                        f'Rejected GPS position jump: {jump:.2f} m '
                        f'(count={self.rejected_fix_count})')
                return
        self.rejected_fix_count = 0
        self.position = candidate_position
        self.position_time = now
        self.longitude = float(msg.longitude)
        self.latitude = float(msg.latitude)
        if not self.route_anchored:
            origin_x, origin_y = self.position
            self.relative_origin = (origin_x, origin_y)
            self.points = [
                (origin_x + x, origin_y + y, speed)
                for x, y, speed in self.points
            ]
            self.route_anchored = True
            self.publish_reference_path()
            self.get_logger().warning(
                'Relative route anchored at first valid GPS fix: '
                f'{origin_x:.3f}, {origin_y:.3f}')
            self.align_initial_heading_if_ready()
        self.fix_time = now

    def velocity_callback(self, msg):
        east = float(msg.twist.linear.x)
        north = float(msg.twist.linear.y)
        if math.isfinite(east) and math.isfinite(north):
            self.velocity = (east, north)
            now = time.monotonic()
            previous_velocity_time = self.velocity_time
            self.velocity_time = now
            speed = math.hypot(east, north)
            self.gps_course_speed = speed
            if speed >= self.min_course_speed:
                measured_course = math.atan2(north, east)
                dt = now - previous_velocity_time
                self.gps_course = rate_limit_angle(
                    self.gps_course, measured_course, dt,
                    self.max_course_rate)
                self.gps_course_time = now
                self.align_imu_to_course(self.gps_course)
            if (self.relative_route and self.route_anchored
                    and not self.relative_course_aligned
                    and speed >= self.min_course_speed):
                self.align_relative_route_to_course(
                    self.gps_course)
            if (self.learn_heading_from_course
                    and speed >= self.min_course_speed
                    and self.imu_yaw is not None):
                course = self.gps_course
                measured_offset = wrap_angle(course - self.imu_yaw)
                if self.heading_offset is None:
                    self.heading_offset = measured_offset
                else:
                    delta = wrap_angle(measured_offset - self.heading_offset)
                    self.heading_offset = wrap_angle(
                        self.heading_offset + 0.10 * delta)

    def align_relative_route_to_course(self, course):
        """Rotate an anchored relative route to the measured travel course."""
        if self.relative_origin is None or len(self.points) < 2:
            return
        ox, oy = self.relative_origin
        x0, y0, _ = self.points[0]
        x1, y1, _ = self.points[1]
        route_heading = math.atan2(y1 - y0, x1 - x0)
        rotation = wrap_angle(course - route_heading)
        cosine, sine = math.cos(rotation), math.sin(rotation)
        rotated = []
        for x, y, speed in self.points:
            dx, dy = x - ox, y - oy
            rotated.append((
                ox + cosine * dx - sine * dy,
                oy + sine * dx + cosine * dy,
                speed))
        self.points = rotated
        self.relative_course_aligned = True
        self.initial_heading_aligned = True
        if self.imu_yaw is not None:
            self.heading_offset = wrap_angle(course - self.imu_yaw)
        self.publish_reference_path()
        self.get_logger().warning(
            'Relative route rotated to measured startup course; '
            f'rotation={rotation:.3f} rad. AUTO is now ready')

    def imu_callback(self, msg):
        q = msg.orientation
        if all(math.isfinite(value) for value in (q.x, q.y, q.z, q.w)):
            self.imu_yaw = quaternion_yaw(q)
            self.imu_time = time.monotonic()
            self.align_initial_heading_if_ready()

    def align_imu_to_course(self, course):
        """Tie the IMU yaw to GPS course so heading survives low speed.

        Without this, heading_offset is only ever set for relative routes, so
        on an absolute route the corrected_imu branch is unreachable and any
        GPS-course dropout falls straight through to the route tangent. The
        tangent is a guess, not a measurement, and switching back from it steps
        the heading by tens of degrees, which the controller turns into a large
        steering jolt. Tracking the offset slowly also follows IMU drift.
        """
        if self.imu_yaw is None:
            return
        offset = wrap_angle(course - self.imu_yaw)
        if self.heading_offset is None:
            self.heading_offset = offset
            self.get_logger().warning(
                f'IMU heading aligned to GPS course; offset={offset:.3f} rad '
                f'({math.degrees(offset):.1f} deg from the assumed '
                f'{self.imu_yaw_offset:.3f}). A large value here means the '
                'compass heading is not trustworthy at a standstill.')
            return
        error = wrap_angle(offset - self.heading_offset)
        self.heading_offset = wrap_angle(
            self.heading_offset + self.imu_align_gain * error)

    def align_initial_heading_if_ready(self):
        """Treat the vehicle's startup pose as aligned to a relative route."""
        if (self.initial_heading_aligned or not self.align_relative_heading
                or not self.relative_route or not self.route_anchored
                or self.imu_yaw is None or len(self.points) < 2):
            return
        x0, y0, _ = self.points[0]
        x1, y1, _ = self.points[1]
        route_heading = math.atan2(y1 - y0, x1 - x0)
        self.heading_offset = wrap_angle(route_heading - self.imu_yaw)
        self.initial_heading_aligned = True
        self.get_logger().warning(
            'Relative-route heading aligned to startup vehicle pose; '
            f'yaw offset={self.heading_offset:.3f} rad')

    def current_heading(self, now):
        if (self.relative_route and self.require_relative_course_alignment
                and not self.relative_course_aligned):
            return None
        if (self.gps_course is not None
                and now - self.gps_course_time <= self.gps_course_hold
                and not (self.heading_reverse
                         and self.gps_course_speed < self.reverse_course_speed)):
            # gps_course 는 '움직이는 방향' 이라 전진이든 후진이든 그대로
            # 쓸 수 있다. 다만 후진 속도대에서는 잡음이 커서 걸러낸다.
            self.heading_source = 'gps_course'
            return self.gps_course
        # Only trust the IMU once GPS course has tied it to a real bearing.
        # Assuming the MAVROS yaw was already an absolute ENU heading (offset
        # 0) proved wrong on this vehicle by about 76 degrees, so at a standing
        # start the follower steered hard the wrong way and then snapped back
        # the moment course arrived. align_imu_to_course() sets the offset on
        # the first valid course and tracks drift from there.
        if (self.use_imu_heading and self.heading_offset is not None
                and self.imu_yaw is not None
                and now - self.imu_time <= self.sensor_timeout):
            offset = (
                self.heading_offset
                if self.heading_offset is not None else self.imu_yaw_offset)
            # IMU yaw 는 '차체가 향한 방향' 이다. 후진 중에는 진행 방향이
            # 그 반대이므로 180도를 더해야 gps_course 와 같은 뜻이 된다.
            # 이 한 줄이 빠지면 IMU 를 켰을 때 후진에서 정확히 반대로 꺾는다.
            self.heading_source = 'corrected_imu'
            heading = wrap_angle(self.imu_yaw + offset)
            return wrap_angle(heading + math.pi) if self.heading_reverse else heading
        if self.allow_route_heading and self.route_anchored:
            index = min(self.nearest_index, len(self.points) - 2)
            x0, y0, _ = self.points[index]
            x1, y1, _ = self.points[index + 1]
            if not self.route_heading_warning_sent:
                self.get_logger().warning(
                    'Heading sensor unavailable; using route tangent until '
                    'GPS course becomes available')
                self.route_heading_warning_sent = True
            self.heading_source = 'route_tangent'
            return math.atan2(y1 - y0, x1 - x0)
        self.heading_source = 'none'
        return None

    def wheel_speed_callback(self, msg):
        """엔코더 속도. 기록 전용이고 주행 판단에는 관여하지 않는다."""
        value = float(msg.data)
        if math.isfinite(value):
            self.wheel_speed = value
            self.wheel_speed_time = time.monotonic()

    def direction_run(self, index, reverse):
        """index 가 속한 같은 방향 구간의 (시작, 끝) 색인."""
        return direction_run_bounds(self.cusp_reverse, index, reverse)

    def build_cusp_reverse(self):
        if self.allow_reverse and self.reverse_waypoint_ranges:
            flags, self.cusp_indices = reverse_flags_from_ranges(
                len(self.points), self.reverse_waypoint_ranges)
            self.get_logger().warning(
                f'Explicit REVERSE segments: {self.reverse_waypoint_ranges}')
            return flags
        if not self.allow_reverse or self.reverse_cusp <= 0.0:
            self.cusp_indices = []
            return [False] * len(self.points)
        flags, self.cusp_indices = directions_from_cusps(
            self.points, self.reverse_cusp)
        runs = []
        for index, value in enumerate(flags):
            if value and runs and index == runs[-1][1] + 1:
                runs[-1][1] = index
            elif value:
                runs.append([index, index])
        if runs:
            self.get_logger().warning(
                'Cusps over %.0f deg at waypoints %s; REVERSE segments %s'
                % (math.degrees(self.reverse_cusp),
                   ', '.join(str(i) for i in self.cusp_indices),
                   ', '.join('%d~%d' % (a, b) for a, b in runs)))
        return flags

    def stop(self, reason):
        self.filtered_steering = 0.0
        self.last_control_time = time.monotonic()
        self.command_pub.publish(Twist())
        if reason != self.last_reason:
            self.get_logger().warning('GPS follower STOP: ' + reason)
            self.last_reason = reason

    def write_tracking_row(self, now, heading, waypoint_index, route_error,
                           lookahead, target_index, target_x, target_y,
                           alpha, raw_steering, command_speed, state):
        if self.tracking_log_writer is None or self.position is None:
            return
        self.tracking_log_writer.writerow({
            'elapsed_sec': f'{now - self.tracking_start_time:.6f}',
            'longitude': '' if self.longitude is None else f'{self.longitude:.12f}',
            'latitude': '' if self.latitude is None else f'{self.latitude:.12f}',
            'utm_easting_m': f'{self.position[0]:.3f}',
            'utm_northing_m': f'{self.position[1]:.3f}',
            'speed_mps': f'{math.hypot(*self.velocity):.3f}',
            'heading_rad': '' if heading is None else f'{heading:.6f}',
            'heading_source': self.heading_source,
            'waypoint_index': waypoint_index,
            'route_error_m': '' if route_error is None else f'{route_error:.3f}',
            'lookahead_m': '' if lookahead is None else f'{lookahead:.3f}',
            'target_index': '' if target_index is None else target_index,
            'target_easting_m': '' if target_x is None else f'{target_x:.3f}',
            'target_northing_m': '' if target_y is None else f'{target_y:.3f}',
            'alpha_rad': '' if alpha is None else f'{alpha:.6f}',
            'raw_steering_rad': (
                '' if raw_steering is None else f'{raw_steering:.6f}'),
            'filtered_steering_rad': f'{self.filtered_steering:.6f}',
            'command_speed_mps': f'{command_speed:.4f}',
            'wheel_speed_mps': (
                '' if self.wheel_speed is None
                or now - self.wheel_speed_time > self.sensor_timeout
                else f'{self.wheel_speed:.3f}'),
            'speed_error_mps': (
                '' if self.wheel_speed is None
                or now - self.wheel_speed_time > self.sensor_timeout
                else f'{math.hypot(*self.velocity) - self.wheel_speed:.3f}'),
            'state': state,
        })
        self.tracking_log_stream.flush()

    def control_tick(self):
        now = time.monotonic()
        if self.control_mode != 'GPS':
            # GPS tracking must never generate stop/route-error commands while
            # the physical transmitter or another mode owns the vehicle.
            return
        if self.position is None or now - self.fix_time > self.sensor_timeout:
            return self.stop('GPS fix missing or stale')
        # heading 을 구하기 전에 지금 후진 중인지 알려 준다. 방위원마다
        # '진행 방향' 으로 바꾸는 방식이 다르기 때문이다.
        self.heading_reverse = self.driving_reverse
        heading = self.current_heading(now)
        if heading is None:
            if (self.relative_route and self.require_relative_course_alignment
                    and not self.relative_course_aligned):
                return self.stop(
                    'relative route direction unknown; drive straight in MANUAL '
                    'above course threshold')
            return self.stop('heading unavailable; move manually above course threshold')

        x, y = self.position
        nearest = bounded_nearest_index(
            self.points, x, y, self.nearest_index,
            self.nearest_initialized,
            self.nearest_search_backward,
            self.nearest_search_forward,
            self.maximum_waypoint_advance)
        route_error = math.hypot(
            self.points[nearest][0] - x, self.points[nearest][1] - y)
        # 기어 전환 중에는 색인을 얼린다. 이 차는 제동이 없어서 0 을
        # 명령해도 관성으로 몇 m 를 더 간다. 그 사이 색인이 앞으로 밀리면
        # 서고 났을 때 후진 구간을 이미 지나쳐 있다. 전환이 끝날 때까지
        # 전환을 시작한 웨이포인트에 붙잡아 둔다.
        if self.shift_pending_direction is not None:
            if self.shift_hold_index is None:
                self.shift_hold_index = nearest
            nearest = self.shift_hold_index
        elif now - self.shift_time < self.shift_settle:
            # 전환 직후에는 색인을 묶는다. 관성으로 굴러가는 사이 색인이
            # 앞으로 밀리면 후진 구간을 지나쳐 버린다.
            if self.shift_hold_index is None:
                self.shift_hold_index = nearest
            nearest = self.shift_hold_index
        elif self.active_run is not None:
            self.shift_hold_index = None
            # 전진/후진 구간의 끝을 넘지 못하게 막는다. 주차 경로는 자기
            # 위로 되돌아와서, 막지 않으면 후진 도중에 탈출 구간 점이 더
            # 가깝게 잡혀 색인이 건너뛴다.
            nearest = max(self.active_run[0], min(nearest, self.active_run[1]))
        self.nearest_index = nearest
        # 후진 여부는 목표점이 아니라 '지금 서 있는' 웨이포인트로 정한다.
        # 목표점으로 정하면 LAD 만큼 미리 후진으로 바뀌어 버린다.
        segment_speed = self.points[nearest][2]
        # LAD 목표점 하나만 보지 않는다. 경로 앞쪽에서 가장 가까운
        # 첨점을 찾아, 제동 거리 안에 들어오면 그 너머의 방향을 지금부터
        # 쓴다. 그래야 도착 전에 제동이 시작되어 첨점을 지나치지 않는다.
        look = nearest
        for cusp in self.cusp_indices:
            if cusp < nearest:
                continue
            # 첨점에 다 왔거나(제동 거리 안) 이미 도달했으면 그 너머의
            # 방향을 쓴다. 거리만 보면, 첨점을 지나쳐 더 간 뒤에는 거리가
            # 다시 멀어져 전환 판정이 사라진다. 그러면 방향이 영영 안
            # 바뀌어 계속 같은 쪽으로 간다.
            # cusp 자체는 이전 방향으로 도착하는 정지점이다. 따라서
            # 선행 제동을 끈 경로(cusp_brake_distance <= 0)는 cusp 다음
            # 웨이포인트부터 새 방향을 적용한다. 예: cusp 13이면 WP14에서
            # 정지/후진 전환. 예전의 nearest >= cusp는 WP13, 현장에서는
            # 최근접 색인이 빨리 잡혀 WP12에서도 후진 전환을 일으켰다.
            prebrake = (
                self.cusp_brake_distance > 0.0
                and math.hypot(self.points[cusp][0] - x,
                               self.points[cusp][1] - y)
                <= self.cusp_brake_distance)
            # 양수 제동 거리를 명시한 주차 경로에서는 최근접 색인이 cusp에
            # 도달한 것 자체도 전환 조건이다. 되돌아가는 다음 점은 현재
            # 진행방향 뒤에 있으므로, cusp를 지나 계속 직진해도 nearest가
            # 다음 색인으로 절대 넘어가지 않을 수 있다.
            reached_cusp = (
                self.cusp_brake_distance > 0.0 and nearest == cusp)
            if nearest > cusp or prebrake or reached_cusp:
                look = min(cusp + 1, len(self.points) - 1)
            break
        explicit_reverse_start = (
            self.allow_reverse
            and not self.reverse_start_consumed
            and 0 <= self.reverse_start_waypoint < len(self.points)
            and nearest >= self.reverse_start_waypoint)
        # WP11은 경로 정의상 마지막 '전진' 점이다. 여기서 후진 전환을
        # 확정한 직후에도 최근접점은 잠시 WP11에 남으므로 cusp flag만
        # 다시 보면 다음 틱에 곧바로 전진으로 되돌아간다. 시작한 후진
        # run은 종료점(WP16) 처리 전까지 반드시 래치한다.
        in_latched_reverse_run = (
            self.driving_reverse
            and self.active_run is not None
            and self.active_run[0] <= nearest <= self.active_run[1])
        want_reverse = self.allow_reverse and (
            segment_speed < 0.0
            or self.cusp_reverse[look]
            or explicit_reverse_start
            or in_latched_reverse_run)
        # 방향 전환이 결정된 바로 이 틱도 조향 중립이어야 한다. 실제
        # driving_reverse/shift_time 갱신은 명령 생성부에서 이뤄지므로,
        # shift_time만 검사하면 첫 전환 명령 한 번은 이전 조향이 나간다.
        shifting_now = (
            self.shift_pending_direction is not None
            or want_reverse != self.driving_reverse)

        # ----- 후진 구간 끝 -----
        # 후진 구간의 다음 점(= 칸 안쪽 목표)을 지나쳤는지 평면으로 잰다.
        # 지나쳤으면 더 뒤로 가지 않는다. 위치가 조금 안 맞아도 거기서
        # 세우고 주차로 친 뒤, 구간 다음 점부터 전진으로 이어간다.
        if self.driving_reverse and self.active_run is not None:
            # 평면의 법선은 '후진 마지막 구간' 방향이어야 한다. 그 다음
            # 점(탈출 구간 첫 점) 방향을 쓰면 방향이 전혀 달라서, 후진을
            # 시작하는 자리가 이미 평면 너머로 잡힌다.
            end = self.active_run[1]
            # 평면이 지나는 점은 지정한 정지 웨이포인트, 없으면 구간 끝.
            goal = end
            if 0 <= self.reverse_stop_waypoint < len(self.points):
                goal = self.reverse_stop_waypoint
            # 법선은 언제나 '후진 마지막 구간' 방향이다. 평면 위치만
            # 바뀌고 어느 쪽이 '지났다' 인지는 그대로여야 한다.
            previous = max(0, end - 1)
            dx = self.points[end][0] - self.points[previous][0]
            dy = self.points[end][1] - self.points[previous][1]
            length = math.hypot(dx, dy)
            if length > 1e-6:
                past = ((x - self.points[goal][0]) * dx
                        + (y - self.points[goal][1]) * dy) / length
                if past >= self.reverse_overshoot and self.park_phase is None:
                    self.park_phase = 'brake'
                    # 재개 지점을 지금 확정한다. 나중에 active_run 을 보면
                    # 일반 전환 블록이 이미 전진 구간으로 덮어쓴 뒤라
                    # 경로 끝 색인을 집어 버린다.
                    # 정지한 점 바로 다음부터 다시 추종한다.
                    self.park_resume = min(goal + 1, len(self.points) - 1)
                    self.get_logger().warning(
                        'Reverse reached waypoint %d (%.2f m past); stopping '
                        'here and treating it as parked' % (goal, past))

        if self.park_phase is not None:
            want_reverse = False
            if self.park_phase == 'brake':
                motion = self.wheel_speed
                if (motion is None
                        or now - self.wheel_speed_time > self.sensor_timeout):
                    motion = -math.hypot(*self.velocity)
                if motion > -self.reverse_switch_speed:
                    self.park_phase = 'hold'
                    self.park_until = now + self.waypoint_hold
                    self.get_logger().warning(
                        'Parked; holding %.1f s then driving forward'
                        % self.waypoint_hold)
            elif now >= self.park_until:
                # 후진 구간 끝(21) 다음다음 점(23)부터 다시 추종한다.
                # 22 는 방금 선 자리라 건너뛴다. 거기 걸린 정지
                # 웨이포인트도 이미 치른 것으로 본다.
                resume = self.park_resume
                self.nearest_index = resume
                nearest = resume
                segment_speed = self.points[resume][2]
                self.driving_reverse = False
                want_reverse = False
                self.active_run = self.direction_run(resume, False)
                self.hold_index = resume
                self.shift_hold_index = resume
                self.shift_time = now
                self.park_phase = None
                self.get_logger().warning(
                    'Resuming forward tracking from waypoint %d (%.2f m/s)'
                    % (resume, abs(segment_speed)))
        # 아직 방향이 안 바뀐 채로 관성으로 굴러가는 중인가. 이 동안은
        # 조향을 중립으로 고정한다. 목표 방향 기준으로 계산한 조향을
        # 그대로 주면 차가 아직 반대로 움직이는 중이라 그 각도가 정확히
        # 반대로 작용해 차머리가 홱 돌아간다.
        # 방향 전환은 미루지 않는다. 반대 부호 명령을 바로 내보내야
        # 펌웨어 AUTO 제동이 역토크로 세운다. follower 가 '아직 반대로
        # 가는 중인지' 를 판정해 미루려 했더니, 그 판정이 틀리면 제동
        # 자체가 안 나가거나 조향이 0 으로 묶인 채 계속 굴러갔다.
        # 대신 전환 직후 잠깐만 조향을 중립으로 잡는다. 시간 기준이라
        # 어떤 경우에도 교착되지 않는다.
        # 무장 직후 startup_straight 초 동안은 조향을 0 으로 묶고 직진한다.
        # 그 사이에는 매 틱 경로 전체에서 최근접점을 다시 찾는다. 색인을
        # 여기서 바로 고정하면 아직 heading 도 없고 차도 안 움직인 상태의
        # 점에 물려 버린다. 직진이 끝나 GPS course 가 살아난 뒤의 위치에서
        # 잡아야 맞다. 고정된 뒤부터 bounded 탐색이 단조로 동작한다.
        in_startup = (self.armed_time is not None
                      and now - self.armed_time < self.startup_straight)
        self.nearest_initialized = not in_startup
        if not in_startup and not self.startup_locked:
            self.startup_locked = True
            self.get_logger().warning(
                f'Startup straight done; locked on to waypoint {nearest} '
                f'at {route_error:.2f} m')
        self.index_pub.publish(UInt32(data=self.nearest_index))
        if self.nearest_index != self.last_logged_waypoint:
            self.get_logger().info(
                f'Current waypoint: {self.nearest_index}')
            self.last_logged_waypoint = self.nearest_index
        self.error_pub.publish(Float32(data=float(route_error)))
        if route_error > self.max_route_error:
            self.write_tracking_row(
                now, heading, self.nearest_index, route_error,
                None, None, None, None, None, None, 0.0,
                'STOP_ROUTE_ERROR')
            return self.stop(
                f'route error {route_error:.2f} m exceeds limit')

        final_distance = math.hypot(
            self.points[-1][0] - x, self.points[-1][1] - y)
        goal_reached = (
            self.nearest_index >= len(self.points) - 2
            and final_distance <= self.goal_tolerance)
        self.goal_pub.publish(Bool(data=goal_reached))
        if goal_reached:
            return self.stop('final waypoint reached')

        measured_speed = math.hypot(*self.velocity)
        if self.use_speed_banded_lookahead:
            lookahead = speed_banded_lookahead(
                measured_speed,
                self.lookahead_speed_limits,
                self.lookahead_by_speed)
        else:
            lookahead = self.min_lookahead + self.lookahead_gain * measured_speed
        lookahead = lookahead_for_waypoint(
            self.nearest_index, self.lookahead_override_ranges,
            self.lookahead_override, lookahead)
        if want_reverse and self.reverse_lookahead >= 0.0:
            # 후진 LAD.  >0 이면 그 값으로 고정, 0 이면 LAD 를 쓰지 않고
            # 바로 다음 점을 겨눈다. 음수면 전진과 같은 속도 밴드 LAD 를
            # 그대로 쓴다.
            lookahead = self.reverse_lookahead
        self.lookahead_pub.publish(Float32(data=float(lookahead)))
        if want_reverse and self.reverse_lookahead == 0.0:
            last_index = len(self.points) - 1
            target_run = self.active_run
            if target_run is None:
                target_run = self.direction_run(
                    self.nearest_index, want_reverse)
            if target_run is not None:
                last_index = min(last_index, target_run[1])
            target_index = min(self.nearest_index + 1, last_index)
        else:
            # 목표점도 현재 방향 구간을 벗어나면 안 된다. 후진 구간
            # 끝자락에서 LAD 가 길면 목표가 탈출 구간으로 넘어가고,
            # 그러면 후진 중에 전진 구간 점을 겨누게 된다.
            last_index = len(self.points) - 1
            target_run = self.active_run
            if target_run is None:
                target_run = self.direction_run(
                    self.nearest_index, want_reverse)
            if target_run is not None:
                last_index = min(last_index, target_run[1])
            target_index = self.nearest_index
            while target_index < last_index:
                tx, ty, _ = self.points[target_index]
                if math.hypot(tx - x, ty - y) >= lookahead:
                    break
                target_index += 1
        tx, ty, recorded_speed = self.points[target_index]
        target_bearing = math.atan2(ty - y, tx - x)
        alpha = wrap_angle(target_bearing - heading)
        distance = max(0.1, math.hypot(tx - x, ty - y))
        steering_left = math.atan2(
            2.0 * self.wheelbase * math.sin(alpha), distance)
        steering = -steering_left if self.positive_right else steering_left
        if want_reverse and self.invert_reverse_steering:
            # 후진 pure pursuit. heading(gps_course 든 route_tangent 이든)
            # 은 '진행 방향' 이라 후진 중에도 그대로 쓸 수 있다. 다만
            # 조향은 부호가 뒤집힌다. 앞바퀴가 뒤따라가는 꼴이 되기
            # 때문이다. 이 한 줄이 빠지면 차가 경로 반대로 꺾으며 발산한다.
            steering = -steering
        steering = max(-self.max_steer, min(self.max_steer, steering))

        # The route tangent is an assumption about where the vehicle points,
        # not a measurement. Steering hard on it is what threw the vehicle off
        # the path at every start: the moment real GPS course arrived the
        # heading stepped by tens of degrees and the command jumped with it.
        # Creep nearly straight instead until a real heading shows up.
        if (self.armed_time is not None
                and now - self.armed_time < self.startup_straight):
            steering = 0.0

        if (shifting_now
                or now - self.shift_time < self.shift_settle
                or self.park_phase is not None):
            # 전환 직후. 아직 관성으로 반대로 굴러가는 중이라, 목표
            # 방향 기준으로 계산한 조향이 정확히 반대로 작용한다.
            # 주차 정지 처리 중에도 핸들은 중립으로 둔다.
            steering = 0.0
            self.filtered_steering = 0.0

        if self.heading_source == 'route_tangent' and not want_reverse:
            steering = max(-self.route_tangent_max_steer,
                           min(self.route_tangent_max_steer, steering))
        # 후진 구간에서는 클램프를 풀어야 한다. 방향을 바꾼 직후에는 아직
        # 새 course 가 없어 route_tangent 으로 떨어지는데, 거기서 조향이
        # 0 으로 묶이면 주차 조작 자체가 성립하지 않는다. 경로 접선은
        # 진행 방향과 같으므로 후진 중에도 올바른 기준이다.

        # Suppress GPS jitter around centre, then low-pass and rate-limit the
        # command so the steering actuator cannot flick rapidly left/right.
        if abs(steering) < self.steering_deadband:
            steering = 0.0
        filtered = (
            self.steering_filter_alpha * steering
            + (1.0 - self.steering_filter_alpha) * self.filtered_steering)
        dt = max(0.001, min(0.2, now - self.last_control_time))
        max_change = self.max_steering_rate * dt
        change = max(-max_change, min(max_change,
                                      filtered - self.filtered_steering))
        self.filtered_steering = max(
            -self.max_steer,
            min(self.max_steer, self.filtered_steering + change))
        self.last_control_time = now

        command = Twist()
        magnitude = abs(segment_speed) if segment_speed else self.target_speed
        magnitude = min(magnitude, self.target_speed)
        if self.external_speed_limit is not None:
            magnitude = min(magnitude, self.external_speed_limit)
        state = 'TRACKING'
        if (segment_speed == 0.0 and self.waypoint_hold > 0.0
                and self.route_has_speed):
            # 정지 웨이포인트. 처음 닿았을 때 시각을 찍어 두고 그때부터
            # 센다. 서 있는 동안은 nearest 가 안 움직이므로 다시 찍히지
            # 않는다. 시간이 지나면 다음 점의 속도로 출발한다.
            if self.hold_index != nearest:
                self.hold_index = nearest
                self.hold_until = now + self.waypoint_hold
                self.get_logger().warning(
                    'Holding %.1f s at waypoint %d'
                    % (self.waypoint_hold, nearest))
            if now < self.hold_until:
                magnitude = 0.0
                steering = 0.0
                self.filtered_steering = 0.0
                state = 'HOLD'
            else:
                following = self.points[min(nearest + 1, len(self.points) - 1)][2]
                magnitude = min(abs(following) or self.target_speed,
                                self.target_speed)
        commanded_reverse = self.driving_reverse
        shift_estop = self.park_phase is not None
        if (self.park_phase is None
                and self.shift_pending_direction is None
                and want_reverse != self.driving_reverse):
            self.shift_pending_direction = want_reverse
            self.shift_hold_index = nearest
            self.shift_time = now
            self.get_logger().warning(
                'Shift E-stop ACTIVE at waypoint %d; stopping before %s'
                % (nearest, 'REVERSE' if want_reverse else 'FORWARD'))
        if self.shift_pending_direction is not None:
            shift_estop = True
            magnitude = 0.0
            steering = 0.0
            self.filtered_steering = 0.0
            state = 'ESTOP_FOR_SHIFT'
            motion = self.wheel_speed
            if (motion is None
                    or now - self.wheel_speed_time > self.sensor_timeout):
                motion = math.hypot(*self.velocity)
            if (now - self.shift_time >= self.shift_settle
                    and abs(motion) <= self.reverse_switch_speed):
                new_direction = self.shift_pending_direction
                self.driving_reverse = new_direction
                if new_direction and explicit_reverse_start:
                    self.reverse_start_consumed = True
                commanded_reverse = new_direction
                self.active_run = self.direction_run(nearest, new_direction)
                self.shift_pending_direction = None
                self.shift_time = now
                self.gps_course_time = 0.0
                state = 'SHIFT_READY'
                self.get_logger().warning(
                    'Shift E-stop complete at waypoint %d; starting %s '
                    '(speed %.2f m/s)'
                    % (nearest,
                       'REVERSE' if new_direction else 'FORWARD', motion))
        if self.park_phase == 'brake':
            magnitude = 0.0
            commanded_reverse = self.driving_reverse
            steering = 0.0
            self.filtered_steering = 0.0
            state = 'PARK_BRAKE'
        elif self.park_phase == 'hold':
            magnitude = 0.0
            steering = 0.0
            self.filtered_steering = 0.0
            state = 'PARKED'
        elif self.driving_reverse and magnitude > 0.0:
            state = 'REVERSE'
        self.shift_estop_pub.publish(Bool(data=shift_estop))
        # 판정 변수를 그대로 남긴다. w=want_reverse, d=driving_reverse,
        # L=look. 계산과 실제 거동이 어긋날 때 이 세 개면 바로 갈린다.
        state = '%s|w%d d%d L%d' % (state, want_reverse,
                                    self.driving_reverse, look)
        command.linear.x = -magnitude if commanded_reverse else magnitude
        command.angular.z = self.filtered_steering
        self.command_pub.publish(command)
        self.write_tracking_row(
            now, heading, self.nearest_index, route_error,
            lookahead, target_index, tx, ty, alpha, steering,
            command.linear.x, state)
        self.last_reason = None

    def shutdown_stop(self):
        if rclpy.ok():
            for _ in range(3):
                self.command_pub.publish(Twist())
        if self.tracking_log_stream is not None:
            self.tracking_log_stream.flush()
            self.tracking_log_stream.close()
            self.tracking_log_stream = None


def main(args=None):
    rclpy.init(args=args)
    node = GpsPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
