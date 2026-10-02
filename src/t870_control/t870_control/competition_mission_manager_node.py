#!/usr/bin/env python3
"""Waypoint-driven coordinator for the combined competition route."""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String, UInt32


def ramp_speed(waypoint, start, stop, speed_at_start, speed_at_stop):
    if stop <= start:
        return speed_at_stop
    ratio = max(0.0, min(1.0, (waypoint - start) / (stop - start)))
    return speed_at_start + ratio * (speed_at_stop - speed_at_start)


class CompetitionMissionManager(Node):
    def __init__(self):
        super().__init__('competition_mission_manager')
        defaults = {
            't_path_1': '', 't_path_2': '', 'p_path_1': '', 'p_path_2': '',
            'ox_path': '', 'hill_stop_wp': 39, 'timed_estop_wp': 532,
            'estop_duration_sec': 3.0,
            'cruise_speed_mps': 2.2222,
            'lidar_speed_mps': 0.56,
            'traffic_light_topic': '/vision/traffic_light_state',
            # 정지선을 이만큼 지나면 그 신호등은 끝난 것으로 본다.
            # 상한이 없으면 GO 판정을 못 받은 신호등이 영영 남아, 정지선을
            # 한참 지난 뒤에도 적색만 보이면 계속 멈춘다.
            'traffic_release_window_wp': 5,
            # 정지선에서 이 시간 넘게 서 있으면 그냥 출발한다. 서 있는
            # 동안은 WP 가 안 올라가므로 위의 WP 상한으로는 교착을 못
            # 막는다. 검출이 초록을 놓쳐도 코스를 끝낼 수 있게 하는
            # 마지막 안전장치다. 0 이면 끈다(무한 대기).
            'traffic_max_hold_sec': 25.0,
            # 규정 정차 직후 탈출 가속. WP39 는 오르막이라 평소 순항속도로는
            # 다시 붙기 어렵다. 정차가 풀린 순간부터 이 시간 동안만 높은
            # 속도를 허용한다. 끝나면 순항속도로 돌아간다.
            # 약 10 km/h. 펌웨어 MAX_SPEED_KMH(10.0) 바로 아래에 둔다.
            # 2.7778 은 10.00008 km/h 라 펌웨어 constrain 에 잘린다.
            'escape_speed_mps': 2.7777,
            'escape_duration_sec': 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.paths = {name: str(self.get_parameter(name).value) for name in (
            't_path_1', 't_path_2', 'p_path_1', 'p_path_2', 'ox_path')}
        if any(not value for value in self.paths.values()):
            raise ValueError('all competition path parameters must be set')

        self.hill_wp = int(self.get_parameter('hill_stop_wp').value)
        self.timed_estop_wp = int(self.get_parameter('timed_estop_wp').value)
        self.stop_duration = float(
            self.get_parameter('estop_duration_sec').value)
        self.cruise_speed = float(
            self.get_parameter('cruise_speed_mps').value)
        self.lidar_speed = float(
            self.get_parameter('lidar_speed_mps').value)
        self.escape_speed = float(
            self.get_parameter('escape_speed_mps').value)
        self.escape_duration = float(
            self.get_parameter('escape_duration_sec').value)
        self.traffic_release_window = int(
            self.get_parameter('traffic_release_window_wp').value)
        self.traffic_max_hold = float(
            self.get_parameter('traffic_max_hold_sec').value)
        self.route = 'T1'
        self.t_choice = 1
        self.p_choice = 1
        self.ox_straight = False
        self.estop_until = 0.0
        self.escape_until = 0.0
        self.completed = set()
        # 판단 구간에서 본 라벨을 세어 둔다. 한 프레임 튄 것으로 결정하지
        # 않고 구간 전체에서 많이 나온 쪽을 쓴다.
        self.traffic_votes = {n: {'green': 0, 'red': 0} for n in (1, 2, 3)}
        self.traffic_green = False
        self.traffic_label = 'UNKNOWN'
        self.traffic_hold_logged = {}
        self.traffic_hold_start = {}
        # 동적장애물 감시가 끝난 WP. 끝난 뒤로는 남은 구간을 미션 속도로
        # 기어갈 이유가 없다. None 이면 아직 안 끝났다는 뜻이다.
        self.dynamic_done_wp = None
        self.current_waypoint = 0

        self.path_pub = self.create_publisher(String, '/t870/set_gps_path', 10)
        self.estop_pub = self.create_publisher(
            Bool, '/t870/competition_estop', 10)
        self.state_pub = self.create_publisher(
            String, '/t870/competition_state', 10)
        self.speed_pub = self.create_publisher(
            Float32, '/t870/gps_speed_limit', 10)
        self.create_subscription(
            UInt32, '/gps/current_waypoint', self.on_waypoint, 10)
        self.create_subscription(
            UInt32, '/competition/t_parking_choice', self.on_t_choice, 10)
        self.create_subscription(
            UInt32, '/competition/parallel_choice', self.on_p_choice, 10)
        self.create_subscription(
            Bool, '/competition/ox_straight', self.on_ox_straight, 10)
        self.create_subscription(
            Bool, '/t870/dynamic_obstacle_done', self.on_dynamic_done, 10)
        # lane_camera_preview 가 발행하는 토픽이다. 예전에는
        # '/vision/traffic_light' 를 구독했는데 그것을 발행하는
        # traffic_red_detector 는 대회 launch 에서 띄우지 않는다. 그러면
        # traffic_green 이 영원히 False 라 첫 신호등 정지 WP 에서 서서
        # 다시 출발하지 못한다.
        self.create_subscription(
            String, str(self.get_parameter('traffic_light_topic').value),
            self.on_traffic_label, 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().warning('Competition mission manager ready: route=T1')

    def on_t_choice(self, msg):
        if int(msg.data) in (1, 2):
            self.t_choice = int(msg.data)

    def on_p_choice(self, msg):
        if int(msg.data) in (1, 2):
            self.p_choice = int(msg.data)

    def on_ox_straight(self, msg):
        self.ox_straight = bool(msg.data)

    def on_dynamic_done(self, msg):
        if not msg.data or self.dynamic_done_wp is not None:
            return
        self.dynamic_done_wp = self.current_waypoint
        self.get_logger().warning(
            '동적장애물 감시 종료 WP%d. 여기서부터 %d WP 에 걸쳐 순항속도로 '
            '올린다 (원래는 WP%d 까지 미션속도였다)'
            % (self.dynamic_done_wp, self.LIDAR_TAIL_WP,
               self.DYNAMIC_OBSTACLE_END))

    def on_traffic_label(self, msg):
        label = msg.data.strip().upper()
        self.traffic_label = label
        self.traffic_green = label in ('GREEN', 'GO', 'PASS')

    def traffic_vote(self):
        """이번 샘플이 어느 쪽 표인가. 모르면 표를 주지 않는다.

        UNKNOWN 을 적색으로 세면, 신호등을 아예 못 보는 상황에서 표가
        전부 적색이 되어 정지 WP 에서 영영 못 나간다.
        """
        if self.traffic_green:
            return 'green'
        if self.traffic_label in ('RED', 'YELLOW', 'STOP'):
            return 'red'
        return None

    def switch(self, route, path_key, start, reverse_ranges=''):
        self.route = route
        request = (f'{self.paths[path_key]}::start={start}'
                   f'::reverse={reverse_ranges}')
        self.path_pub.publish(String(data=request))
        self.state_pub.publish(String(data=f'PATH:{route}:{start}'))
        self.get_logger().warning(f'PATH SWITCH -> {route} WP{start}')

    def timed_stop(self, key):
        if key in self.completed:
            return
        self.completed.add(key)
        self.estop_until = max(
            self.estop_until, time.monotonic() + self.stop_duration)
        # 정차가 풀리는 순간부터 탈출 가속 창을 연다.
        self.escape_until = self.estop_until + self.escape_duration
        self.get_logger().warning(
            'COMPETITION E-STOP: %s, %.1f sec then escape %.2f m/s '
            '(%.1f km/h) for %.1f sec'
            % (key, self.stop_duration, self.escape_speed,
               self.escape_speed * 3.6, self.escape_duration))

    def traffic(self, number, waypoint, judge_lo, judge_hi, stop_wp):
        key = f'traffic_{number}'
        if key in self.completed:
            return
        votes = self.traffic_votes[number]
        if judge_lo <= waypoint <= judge_hi:
            vote = self.traffic_vote()
            if vote is not None:
                votes[vote] += 1
        if waypoint > stop_wp + self.traffic_release_window:
            # 정지선을 한참 지났다. 수동으로 넘겼든 무엇이든 이 신호등은
            # 끝난 것이다. 여기서 닫지 않으면 이후 전 구간에서 적색만
            # 보이면 계속 멈춘다.
            #
            # [주의] 이 창은 '이미 지나쳐 버린' 신호등만 닫는다. 실제로
            # 정지선에 잡혀 서 있는 동안에는 WP 가 늘지 않으므로 여기에
            # 절대 못 온다. 진짜 정지 상태의 탈출구는 GREEN 검출과 아래
            # traffic_max_hold 강제출발 둘뿐이다.
            self.completed.add(key)
            self.get_logger().warning(
                'Traffic light %d: released at WP%d (past stop line %d; '
                'green %d / red %d, last=%s)'
                % (number, waypoint, stop_wp, votes['green'], votes['red'],
                   self.traffic_label))
            return
        if waypoint >= stop_wp:
            # 판단 구간에서 많이 나온 쪽으로 정한다. 표가 하나도 없으면
            # (신호등을 못 봤으면) 통과로 본다. 그래야 교착되지 않는다.
            red_majority = votes['red'] > votes['green']
            if red_majority and not self.traffic_green:
                now = time.monotonic()
                started = self.traffic_hold_start.setdefault(number, now)
                if 0.0 < self.traffic_max_hold <= now - started:
                    # 너무 오래 서 있다. 검출이 초록을 놓쳤을 수 있다.
                    # 코스를 끝내기 위해 출발한다.
                    self.completed.add(key)
                    self.get_logger().warning(
                        'Traffic light %d: FORCED GO after %.0f s at WP%d '
                        '(green %d / red %d, last=%s). 검출이 초록을 '
                        '놓쳤을 수 있다'
                        % (number, now - started, waypoint, votes['green'],
                           votes['red'], self.traffic_label))
                    return
                self.estop_until = max(self.estop_until, now + 0.15)
                # 라벨이 RED 로 굳으면 로그가 한 줄 나오고 조용해진다.
                # 그러면 현장에서 '잡고 있는 중'인지 '죽은' 것인지 알 수
                # 없다. 남은 강제출발 시간을 2 초마다 같이 찍는다.
                held = now - started
                stale = now - self.traffic_hold_logged.get(number, (None, 0.0))[1]
                if (self.traffic_hold_logged.get(number, (None,))[0]
                        != self.traffic_label or stale >= 2.0):
                    self.traffic_hold_logged[number] = (self.traffic_label, now)
                    remain = ('%.0f s' % max(0.0, self.traffic_max_hold - held)
                              if self.traffic_max_hold > 0.0 else '무제한')
                    self.get_logger().warning(
                        'Traffic light %d: holding at WP%d, seeing %s '
                        '(green %d / red %d, %.0f s 경과, 강제출발까지 %s)'
                        % (number, waypoint, self.traffic_label,
                           votes['green'], votes['red'], held, remain))
            else:
                self.completed.add(key)
                self.get_logger().warning(
                    'Traffic light %d: GO at WP%d (green %d / red %d, last=%s)'
                    % (number, waypoint, votes['green'], votes['red'],
                       self.traffic_label))

    def on_waypoint(self, msg):
        wp = int(msg.data)
        self.current_waypoint = wp
        # 음수면 그 정차를 끈다. 가드가 없으면 -1 일 때 wp >= -1 이 항상
        # 참이라 출발하자마자 걸린다.
        if self.hill_wp >= 0 and wp >= self.hill_wp:
            self.timed_stop('hill_wp39')

        for number, (judge_lo, judge_hi, stop_wp) in enumerate(
                self.TRAFFIC_LIGHTS, start=1):
            self.traffic(number, wp, judge_lo, judge_hi, stop_wp)

        if self.route == 'T1' and wp >= 299 and self.t_choice == 2:
            self.switch('T2', 't_path_2', 299, '309:321')
        if self.route == 'T1' and wp >= 330:
            self.switch('P1', 'p_path_1', 335, '641:647')
        elif self.route == 'T2' and wp >= 335:
            self.switch('P1', 'p_path_1', 335, '641:647')

        if (self.timed_estop_wp >= 0
                and self.route == 'P1' and wp >= self.timed_estop_wp):
            self.timed_stop('estop_wp532')
        if self.route == 'P1' and wp >= 638 and self.p_choice == 2:
            self.switch('P2', 'p_path_2', 638, '649:655')
        if self.route in ('P1', 'P2') and wp >= 693 and self.ox_straight:
            self.switch('OX', 'ox_path', 697, '')

    def tick(self):
        active = time.monotonic() < self.estop_until
        self.estop_pub.publish(Bool(data=active))
        self.speed_pub.publish(Float32(data=float(self.speed_limit())))

    # 신호등 판단 구간과 정지선. (판단 시작, 판단 끝, 정지 WP)
    #
    # [2026-09-20] 2번 신호등(판단 262~267, 정지선 268)을 뺐다. 이 코스에서
    # 쓰지 않는다. 빼면 그 구간의 감속/가속 램프도 같이 사라진다.
    #
    # [주의] 로그의 번호는 이 표의 순서다. 이제
    #   'Traffic light 1' = WP147,  'Traffic light 2' = WP427
    # 이다. 예전 로그의 3번이 지금 2번이다.
    TRAFFIC_LIGHTS = ((141, 146, 147), (421, 426, 427))

    # 라이다를 켜는 미션. (라이다 ON WP, 미션 끝 WP, 적용 경로)
    # 경로가 None 이면 어느 경로에서든 적용한다.
    #
    # 이 값은 launch 의 구간 설정과 같아야 한다.
    #   201:227  avoid_ranges                (정적장애물)
    #   297      competition_t_selector      observation_start_waypoint
    #   531:588  estop_ranges                (동적장애물)
    #   632      competition_parallel_selector observation_start_waypoint
    # test_lidar_mission_table_matches_launch 가 대조한다.
    LIDAR_MISSIONS = (
        # [2026-09-19] 200 -> 201. 실차에서 구간에 들어서는 순간
        # 카메라가 풀밭을 정면으로 봤다. 한 WP(약 1 m) 늦게 켜서
        # 그 시점을 지나고 나서 판단하게 한다.
        # [2026-09-19] 끝을 234 -> 227 로 당겼다. 구간이 끝나는 순간
        # mission_mux 가 /cmd_vel/avoid 에서 GPS 로 갈아타는데, 그때
        # 보정각이 남아 있으면 조향이 그만큼 튄다(mode_callback 은 램프
        # 없이 즉시 0 으로 만든다). 장애물을 지나고 여유를 두고 끝낸다.
        (201, 227, None),                  # 정적장애물
        (297, 335, ('T1', 'T2')),          # T주차 판단~조작
        (531, 588, None),                  # 동적장애물
        (632, 693, ('P1', 'P2', 'OX')),    # 평행주차 판단~조작
    )
    # 라이다를 켜기 몇 WP 전부터 감속할지. 라이다가 켜지는 순간에는 이미
    # 미션 속도여야 한다. 8 km/h 로 켜면 판정 프레임도 모자라고, 이 차는
    # 브레이크가 없어 반응해도 늦는다.
    LIDAR_LEAD_WP = 5
    LIDAR_TAIL_WP = 5
    # 클래스 속성으로 둔 것은 테스트가 __init__ 을 건너뛰고 객체를 만들기
    # 때문이다. 인스턴스에서 덮어쓴다.
    dynamic_done_wp = None
    # 동적장애물 미션의 시작/끝. 위 표와 반드시 같아야 한다. 이 미션만
    # 따로 다루는 이유는 emergency_stop_node 가 장애물을 한 번 보내고 나면
    # 스스로 감시를 끄기 때문이다. 그 뒤로는 라이다를 아예 안 보는데
    # 남은 구간을 계속 2 km/h 로 가면 복귀가 한참 걸린다.
    DYNAMIC_OBSTACLE_ON = 531
    DYNAMIC_OBSTACLE_END = 588

    def speed_limit(self):
        """Five-waypoint ramps around LiDAR mission zones."""
        wp = self.current_waypoint
        fast = self.cruise_speed
        slow = self.lidar_speed

        # 신호등 정지선. 이 차는 브레이크가 없어 관성으로 선다. 8 km/h 로
        # 정지선에 들어가면 펌웨어 역토크만으로는 넘어갈 수 있다. 미리
        # 줄여서 들어가고 통과한 뒤 다시 올린다. 판단 구간에서 느려지는
        # 것은 오히려 유리하다. 프레임이 더 쌓인다.
        for _judge_lo, _judge_hi, stop_wp in self.TRAFFIC_LIGHTS:
            if stop_wp - 5 <= wp < stop_wp:
                return ramp_speed(wp, stop_wp - 5, stop_wp, fast, slow)
            if stop_wp <= wp <= stop_wp + 4:
                return ramp_speed(wp, stop_wp, stop_wp + 4, slow, fast)

        # 라이다 미션. 켜지기 LIDAR_LEAD_WP 전부터 줄여서, 켜지는
        # 순간에는 이미 미션 속도로 들어간다.
        for on_wp, end_wp, routes in self.LIDAR_MISSIONS:
            if routes is not None and self.route not in routes:
                continue
            lead = on_wp - self.LIDAR_LEAD_WP
            if lead <= wp < on_wp:
                return ramp_speed(wp, lead, on_wp, fast, slow)
            if on_wp <= wp <= end_wp:
                # 동적장애물은 감시가 끝나면 남은 구간을 기어갈 이유가 없다.
                if (on_wp == self.DYNAMIC_OBSTACLE_ON
                        and self.dynamic_done_wp is not None):
                    tail = self.dynamic_done_wp + self.LIDAR_TAIL_WP
                    if wp >= tail:
                        break            # 표 밖으로 나가 순항속도를 쓴다
                    return ramp_speed(wp, self.dynamic_done_wp, tail,
                                      slow, fast)
                return slow
            if end_wp < wp <= end_wp + self.LIDAR_TAIL_WP:
                return ramp_speed(wp, end_wp, end_wp + self.LIDAR_TAIL_WP,
                                  slow, fast)

        # T주차를 마치면 WP330(T1)/WP335(T2)에서 P1 으로 갈아탄다. 경로가
        # 바뀌는 순간이라 위 표의 꼬리 가속이 걸리지 않는다. 여기서 올린다.
        if self.route in ('P1', 'P2') and 335 <= wp <= 339:
            return ramp_speed(wp, 335, 339, slow, fast)

        # 규정 정차 탈출 가속. 신호등/라이다 구간보다 뒤에 두어 그쪽
        # 제한을 덮지 않게 한다. 오르막에서 다시 붙는 용도다.
        now = time.monotonic()
        if self.estop_until <= now < self.escape_until:
            return self.escape_speed
        return fast


def main(args=None):
    rclpy.init(args=args)
    node = CompetitionMissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.estop_pub.publish(Bool(data=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
