import pytest

from t870_control.competition_mission_manager_node import (
    CompetitionMissionManager, ramp_speed)


def manager_speed(route, waypoint, estop_until=0.0, escape_until=0.0,
                  dynamic_done_wp=None):
    manager = object.__new__(CompetitionMissionManager)
    manager.route = route
    manager.current_waypoint = waypoint
    manager.cruise_speed = 2.2222
    manager.lidar_speed = 0.56
    manager.escape_speed = 2.7777
    manager.estop_until = estop_until
    manager.escape_until = escape_until
    manager.dynamic_done_wp = dynamic_done_wp
    return manager.speed_limit()


def test_dynamic_zone_stays_slow_until_the_obstacle_is_handled():
    """감시가 끝나기 전에는 구간 전체가 미션 속도여야 한다."""
    for waypoint in (531, 550, 570, 588):
        assert manager_speed('P1', waypoint) == pytest.approx(0.56)


def test_dynamic_zone_speeds_up_once_monitoring_ends():
    """장애물을 보내고 나면 남은 구간을 기어갈 이유가 없다.

    emergency_stop_node 는 한 번 서서 보낸 뒤 스스로 감시를 끈다. 그런데도
    미션 매니저가 WP588 까지 2 km/h 로 묶어 복귀가 한참 걸렸다.
    """
    done = 535
    assert manager_speed('P1', done, dynamic_done_wp=done) == pytest.approx(
        0.56)
    # LIDAR_TAIL_WP 에 걸쳐 오른다.
    assert 0.56 < manager_speed('P1', 537, dynamic_done_wp=done) < 2.2222
    assert manager_speed('P1', 540, dynamic_done_wp=done) == pytest.approx(
        2.2222)
    # 그 뒤로는 구간 안이어도 순항속도다.
    assert manager_speed('P1', 570, dynamic_done_wp=done) == pytest.approx(
        2.2222)
    assert manager_speed('P1', 588, dynamic_done_wp=done) == pytest.approx(
        2.2222)


def test_dynamic_release_does_not_touch_other_missions():
    """동적장애물 해제가 주차/정적장애물 구간을 건드리면 안 된다."""
    assert manager_speed('T1', 210, dynamic_done_wp=535) == pytest.approx(0.56)
    assert manager_speed('T1', 300, dynamic_done_wp=535) == pytest.approx(0.56)
    assert manager_speed('P1', 650, dynamic_done_wp=535) == pytest.approx(0.56)


def test_ramp_speed_endpoints():
    assert ramp_speed(195, 195, 200, 2.2222, 0.56) == pytest.approx(2.2222)
    assert ramp_speed(200, 195, 200, 2.2222, 0.56) == pytest.approx(0.56)


def test_static_lidar_speed_profile():
    # [2026-09-19 변경] 미션 시작이 200 -> 201 로 한 WP 밀렸다. 구간에
    # 들어서는 순간 카메라가 풀밭을 정면으로 봤기 때문이다. 감속 시작도
    # 195 -> 196 으로 함께 밀린다(LIDAR_LEAD_WP 5).
    assert manager_speed('T1', 195) == pytest.approx(2.2222)
    assert manager_speed('T1', 198) < 2.2222
    assert manager_speed('T1', 200) < 2.2222      # 아직 램프 중
    assert manager_speed('T1', 201) == pytest.approx(0.56)
    assert manager_speed('T1', 227) == pytest.approx(0.56)
    assert manager_speed('T1', 230) < 2.2222      # 꼬리 램프 중
    assert manager_speed('T1', 232) == pytest.approx(2.2222)


def test_parking_lidar_speed_profiles():
    # [2026-09-19 변경] 감속 시작이 294 -> 292 로 당겨졌다. 라이다 판정이
    # WP297 에서 켜지므로 그 5 WP 전부터 줄인다.
    assert manager_speed('T1', 292) == pytest.approx(2.2222)
    assert manager_speed('T1', 294) < 2.2222
    assert manager_speed('T1', 297) == pytest.approx(0.56)
    assert manager_speed('T1', 299) == pytest.approx(0.56)
    assert manager_speed('P1', 335) == pytest.approx(0.56)
    assert manager_speed('P1', 339) == pytest.approx(2.2222)
    assert manager_speed('P1', 638) == pytest.approx(0.56)
    assert manager_speed('P1', 693) == pytest.approx(0.56)
    assert manager_speed('OX', 697) < 2.2222
    assert manager_speed('OX', 698) == pytest.approx(2.2222)


def make_manager(route='T1'):
    manager = object.__new__(CompetitionMissionManager)
    manager.route = route
    manager.traffic_votes = {n: {'green': 0, 'red': 0} for n in (1, 2, 3)}
    manager.traffic_green = False
    manager.traffic_label = 'UNKNOWN'
    manager.traffic_hold_logged = {}
    manager.traffic_hold_start = {}
    manager.traffic_release_window = 5
    manager.traffic_max_hold = 25.0
    manager.completed = set()
    manager.estop_until = 0.0
    manager.get_logger = lambda: type(
        'L', (), {'warning': lambda _s, *a: None})()
    return manager


class Label:
    def __init__(self, data):
        self.data = data


def test_green_label_clears_traffic_stop():
    """lane_camera_preview 는 'GREEN' 을 보낸다. 그것으로 풀려야 한다."""
    manager = make_manager()
    manager.on_traffic_label(Label('GREEN'))
    assert manager.traffic_green is True
    manager.traffic(1, 147, 141, 146, 147)
    assert 'traffic_1' in manager.completed


def test_red_label_keeps_stop_latched():
    manager = make_manager()
    manager.on_traffic_label(Label('RED'))
    for wp in range(141, 147):
        manager.traffic(1, wp, 141, 146, 147)   # 판단 구간 내내 적색
    manager.traffic(1, 147, 141, 146, 147)      # 정지 WP
    assert 'traffic_1' not in manager.completed
    assert manager.estop_until > 0.0


def test_single_spurious_red_does_not_stop():
    """한 프레임 튄 적색으로 정지하면 안 된다. 최빈값으로 정한다."""
    manager = make_manager()
    manager.on_traffic_label(Label('GREEN'))
    for wp in range(141, 146):
        manager.traffic(1, wp, 141, 146, 147)   # 초록 5표
    manager.on_traffic_label(Label('RED'))
    manager.traffic(1, 146, 141, 146, 147)      # 적색 1표
    manager.traffic(1, 147, 141, 146, 147)
    assert 'traffic_1' in manager.completed
    assert manager.traffic_votes[1] == {'green': 5, 'red': 1}


def test_single_spurious_green_does_not_pass():
    """반대도 같다. 초록 한 번 튀어도 적색 우세면 선다."""
    manager = make_manager()
    manager.on_traffic_label(Label('RED'))
    for wp in range(141, 146):
        manager.traffic(1, wp, 141, 146, 147)   # 적색 5표
    manager.on_traffic_label(Label('GREEN'))
    manager.traffic(1, 146, 141, 146, 147)      # 초록 1표
    manager.on_traffic_label(Label('RED'))
    manager.traffic(1, 147, 141, 146, 147)
    assert 'traffic_1' not in manager.completed


def test_unknown_labels_do_not_vote():
    """UNKNOWN 을 적색으로 세면 신호등을 못 볼 때 영영 못 나간다."""
    manager = make_manager()
    manager.on_traffic_label(Label('UNKNOWN'))
    for wp in range(141, 147):
        manager.traffic(1, wp, 141, 146, 147)
    assert manager.traffic_votes[1] == {'green': 0, 'red': 0}
    manager.traffic(1, 147, 141, 146, 147)
    assert 'traffic_1' in manager.completed


def test_live_green_releases_even_with_red_majority():
    """적색 우세로 섰어도 실제로 초록이 되면 출발한다."""
    manager = make_manager()
    manager.on_traffic_label(Label('RED'))
    for wp in range(141, 147):
        manager.traffic(1, wp, 141, 146, 147)
    manager.traffic(1, 147, 141, 146, 147)
    assert 'traffic_1' not in manager.completed
    manager.on_traffic_label(Label('GREEN'))
    manager.traffic(1, 148, 141, 146, 147)
    assert 'traffic_1' in manager.completed


def test_traffic_light_topic_default_matches_publisher():
    """구독 토픽이 lane_camera_preview 의 발행 토픽과 같아야 한다."""
    import re
    manager_src = open(
        'src/t870_control/t870_control/competition_mission_manager_node.py',
        encoding='utf-8').read()
    preview_src = open(
        'src/t870_control/t870_control/lane_camera_preview.py',
        encoding='utf-8').read()
    default = re.search(r"'traffic_light_topic': '([^']+)'", manager_src)
    assert default, 'traffic_light_topic 기본값을 찾지 못함'
    topic = default.group(1)
    assert f"String, '{topic}'" in preview_src, (
        f'{topic} 을 발행하는 곳이 lane_camera_preview 에 없다')


def test_traffic_stop_line_deceleration():
    """브레이크가 없으므로 정지선에 8 km/h 로 들어가면 안 된다."""
    for _lo, _hi, stop_wp in CompetitionMissionManager.TRAFFIC_LIGHTS:
        assert manager_speed('P1', stop_wp - 6) == pytest.approx(2.2222)
        assert manager_speed('P1', stop_wp - 3) < 2.2222
        assert manager_speed('P1', stop_wp) == pytest.approx(0.56)


def test_traffic_stop_line_reacceleration():
    for _lo, _hi, stop_wp in CompetitionMissionManager.TRAFFIC_LIGHTS:
        assert manager_speed('P1', stop_wp + 2) > 0.56
        assert manager_speed('P1', stop_wp + 4) == pytest.approx(2.2222)


def test_traffic_ramp_is_monotonic_into_stop_line():
    """접근 구간에서 속도가 다시 올라가면 안 된다."""
    for _lo, _hi, stop_wp in CompetitionMissionManager.TRAFFIC_LIGHTS:
        speeds = [manager_speed('P1', wp)
                  for wp in range(stop_wp - 5, stop_wp + 1)]
        assert speeds == sorted(speeds, reverse=True), speeds


def test_traffic_ramps_do_not_touch_lidar_zones():
    """정적장애물/주차 구간 속도를 신호등 램프가 덮어쓰면 안 된다."""
    assert manager_speed('P1', 201) == pytest.approx(0.56)
    assert manager_speed('P1', 227) == pytest.approx(0.56)
    assert manager_speed('T1', 299) == pytest.approx(0.56)
    assert manager_speed('P1', 638) == pytest.approx(0.56)


def test_judgement_window_is_inside_deceleration():
    """판단 구간이 감속 구간과 겹쳐야 프레임이 충분히 쌓인다."""
    for judge_lo, judge_hi, stop_wp in (
            CompetitionMissionManager.TRAFFIC_LIGHTS):
        assert judge_hi < stop_wp
        assert manager_speed('P1', judge_hi) < 2.2222


def test_lidar_on_waypoint_is_already_slow():
    """라이다가 켜지는 WP 에서는 이미 미션 속도여야 한다.

    8 km/h 로 켜면 판정 프레임도 모자라고, 브레이크가 없어 반응해도 늦다.
    """
    for on_wp, _end, routes in CompetitionMissionManager.LIDAR_MISSIONS:
        route = routes[0] if routes else 'T1'
        assert manager_speed(route, on_wp) == pytest.approx(0.56), on_wp


def test_lidar_deceleration_starts_five_waypoints_early():
    lead = CompetitionMissionManager.LIDAR_LEAD_WP
    for on_wp, _end, routes in CompetitionMissionManager.LIDAR_MISSIONS:
        route = routes[0] if routes else 'T1'
        assert manager_speed(route, on_wp - lead - 1) == pytest.approx(2.2222)
        assert manager_speed(route, on_wp - lead) == pytest.approx(2.2222)
        assert manager_speed(route, on_wp - lead + 1) < 2.2222


def test_lidar_zone_stays_slow_throughout():
    for on_wp, end_wp, routes in CompetitionMissionManager.LIDAR_MISSIONS:
        route = routes[0] if routes else 'T1'
        for wp in (on_wp, (on_wp + end_wp) // 2, end_wp):
            assert manager_speed(route, wp) == pytest.approx(0.56), wp


def test_lidar_deceleration_is_monotonic():
    lead = CompetitionMissionManager.LIDAR_LEAD_WP
    for on_wp, _end, routes in CompetitionMissionManager.LIDAR_MISSIONS:
        route = routes[0] if routes else 'T1'
        speeds = [manager_speed(route, wp)
                  for wp in range(on_wp - lead, on_wp + 1)]
        assert speeds == sorted(speeds, reverse=True), (on_wp, speeds)


def test_lidar_mission_table_matches_launch():
    """표의 라이다 ON WP 가 launch 의 실제 설정과 같아야 한다.

    한쪽만 바꾸면 라이다가 8 km/h 에서 켜지거나, 감속만 하고 미션이
    안 켜진다.
    """
    import re
    launch = open('src/t870_control/launch/t870_competition.launch.py',
                  encoding='utf-8').read()
    t_block = re.search(
        "name='competition_t_selector'.*?\\}\\]", launch, re.S).group(0)
    p_block = re.search(
        "name='competition_parallel_selector'.*?\\}\\]", launch, re.S).group(0)
    launch_on = {
        int(re.search(r"'avoid_ranges': '(\d+):", launch).group(1)),
        int(re.search(r"'observation_start_waypoint': (\d+)", t_block).group(1)),
        int(re.search(r"'estop_ranges': '(\d+):", launch).group(1)),
        int(re.search(r"'observation_start_waypoint': (\d+)", p_block).group(1)),
    }
    table_on = {on for on, _end, _routes
                in CompetitionMissionManager.LIDAR_MISSIONS}
    assert table_on == launch_on, (
        f'표 {sorted(table_on)} != launch {sorted(launch_on)}')


def test_t_parking_exit_accelerates_on_p1():
    """T주차를 마치고 P1 으로 갈아탄 뒤 다시 올라가야 한다."""
    assert manager_speed('P1', 335) == pytest.approx(0.56)
    assert manager_speed('P1', 337) > 0.56
    assert manager_speed('P1', 339) == pytest.approx(2.2222)


def test_t2_stays_slow_until_switch_waypoint():
    """T2 는 WP335 에서 갈아탄다. 그전에 가속하면 안 된다."""
    for wp in range(297, 336):
        assert manager_speed('T2', wp) == pytest.approx(0.56), wp


def test_ox_route_keeps_exit_ramp():
    """WP697 에서 OX 로 갈아타도 탈출 가속이 유지되어야 한다."""
    assert manager_speed('OX', 695) > 0.56
    assert manager_speed('OX', 698) == pytest.approx(2.2222)


def test_escape_boost_after_timed_stop():
    """정차가 풀린 직후 3초 동안 10 km/h 를 허용한다.

    WP39 는 오르막이라 순항속도로는 다시 붙기 어렵다.
    """
    import time
    now = time.monotonic()
    # 정차는 끝났고(estop_until 과거), 탈출 창은 아직 열려 있다.
    assert manager_speed('T1', 45, estop_until=now - 0.1,
                         escape_until=now + 2.0) == pytest.approx(2.7777)


def test_no_boost_before_stop_releases():
    """정차 중에는 탈출 속도를 내보내지 않는다."""
    import time
    now = time.monotonic()
    assert manager_speed('T1', 45, estop_until=now + 2.0,
                         escape_until=now + 5.0) == pytest.approx(2.2222)


def test_boost_expires_back_to_cruise():
    import time
    now = time.monotonic()
    assert manager_speed('T1', 45, estop_until=now - 10.0,
                         escape_until=now - 1.0) == pytest.approx(2.2222)


def test_boost_never_overrides_lidar_zone():
    """탈출 창이 열려 있어도 라이다 미션 구간은 미션 속도를 지킨다."""
    import time
    now = time.monotonic()
    for on_wp, _end, routes in CompetitionMissionManager.LIDAR_MISSIONS:
        route = routes[0] if routes else 'T1'
        assert manager_speed(route, on_wp, estop_until=now - 0.1,
                             escape_until=now + 5.0) == pytest.approx(0.56)


def test_boost_never_overrides_traffic_stop_line():
    import time
    now = time.monotonic()
    for _lo, _hi, stop_wp in CompetitionMissionManager.TRAFFIC_LIGHTS:
        assert manager_speed('P1', stop_wp, estop_until=now - 0.1,
                             escape_until=now + 5.0) == pytest.approx(0.56)


def test_escape_speed_within_firmware_limit():
    """펌웨어 MAX_SPEED_KMH 를 넘으면 잘린다."""
    import re
    firmware = open('sketch_aug/sketch_aug.ino', encoding='utf-8').read()
    limit = float(re.search(r'MAX_SPEED_KMH = ([0-9.]+)', firmware).group(1))
    launch = open('src/t870_control/launch/t870_competition.launch.py',
                  encoding='utf-8').read()
    escape = float(re.search(
        r"'gps_escape_speed_mps', default_value='([0-9.]+)'", launch).group(1))
    assert escape * 3.6 <= limit + 1e-6, (
        f'탈출 {escape * 3.6:.1f} km/h > 펌웨어 상한 {limit} km/h')


def test_follower_cap_is_not_below_escape_speed():
    """follower 자체 상한이 탈출속도보다 낮으면 가속이 잘린다."""
    launch = open('src/t870_control/launch/t870_competition.launch.py',
                  encoding='utf-8').read()
    assert "'gps_follow_speed_mps': LaunchConfiguration(\n" \
           "                    'gps_escape_speed_mps')" in launch


def test_traffic_releases_after_passing_stop_line():
    """정지선을 한참 지나면 그 신호등은 끝난 것으로 본다.

    상한이 없으면 GO 를 못 받은 신호등이 영영 남아, 이후 전 구간에서
    적색만 보이면 계속 멈춘다. 실차에서 실제로 그랬다.
    """
    manager = make_manager()
    manager.on_traffic_label(Label('RED'))
    for wp in range(141, 147):
        manager.traffic(1, wp, 141, 146, 147)
    manager.traffic(1, 147, 141, 146, 147)      # 정지선에서 정지
    assert 'traffic_1' not in manager.completed

    # 수동으로 넘겼다. 정지선 + 5 를 넘으면 닫힌다.
    manager.traffic(1, 153, 141, 146, 147)
    assert 'traffic_1' in manager.completed


def test_traffic_still_holds_inside_release_window():
    """정지선 바로 위에서는 계속 기다려야 한다."""
    manager = make_manager()
    manager.on_traffic_label(Label('RED'))
    for wp in range(141, 147):
        manager.traffic(1, wp, 141, 146, 147)
    for wp in (147, 149, 152):                  # 상한(152) 이내
        manager.traffic(1, wp, 141, 146, 147)
        assert 'traffic_1' not in manager.completed, wp
    assert manager.estop_until > 0.0


def test_released_traffic_does_not_stop_later_waypoints():
    """닫힌 뒤에는 적색이 보여도 멈추지 않는다."""
    import time
    manager = make_manager()
    manager.on_traffic_label(Label('RED'))
    for wp in range(141, 147):
        manager.traffic(1, wp, 141, 146, 147)
    manager.traffic(1, 160, 141, 146, 147)      # 상한 넘어 닫힘
    assert 'traffic_1' in manager.completed

    before = manager.estop_until
    for wp in (200, 300, 400):
        manager.traffic(1, wp, 141, 146, 147)
    assert manager.estop_until == before, '닫힌 신호등이 다시 멈췄다'


def test_traffic_forced_go_after_max_hold(monkeypatch):
    """정지선에서 너무 오래 서 있으면 출발한다.

    서 있는 동안은 WP 가 안 올라가므로 WP 상한으로는 교착을 못 막는다.
    검출이 초록을 놓쳐도 코스를 끝낼 수 있어야 한다. 실차에서 초록으로
    바뀌었는데 GREEN 이 한 번도 안 잡혀 못 나간 적이 있다.
    """
    import t870_control.competition_mission_manager_node as module
    clock = [1000.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])

    manager = make_manager()
    manager.on_traffic_label(Label('RED'))
    for wp in range(141, 147):
        manager.traffic(1, wp, 141, 146, 147)

    manager.traffic(1, 147, 141, 146, 147)          # 정지 시작
    assert 'traffic_1' not in manager.completed

    clock[0] += 24.0                                 # 아직 상한 이내
    manager.traffic(1, 147, 141, 146, 147)
    assert 'traffic_1' not in manager.completed

    clock[0] += 2.0                                  # 합계 26 s > 25 s
    manager.traffic(1, 147, 141, 146, 147)
    assert 'traffic_1' in manager.completed


def test_green_still_releases_before_timeout(monkeypatch):
    """제한시간 전에 초록이 오면 그것으로 풀린다."""
    import t870_control.competition_mission_manager_node as module
    clock = [1000.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])

    manager = make_manager()
    manager.on_traffic_label(Label('RED'))
    for wp in range(141, 147):
        manager.traffic(1, wp, 141, 146, 147)
    manager.traffic(1, 147, 141, 146, 147)

    clock[0] += 5.0
    manager.on_traffic_label(Label('GREEN'))
    manager.traffic(1, 147, 141, 146, 147)
    assert 'traffic_1' in manager.completed


def test_traffic_timeout_can_be_disabled(monkeypatch):
    import t870_control.competition_mission_manager_node as module
    clock = [1000.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])

    manager = make_manager()
    manager.traffic_max_hold = 0.0                   # 끔
    manager.on_traffic_label(Label('RED'))
    for wp in range(141, 147):
        manager.traffic(1, wp, 141, 146, 147)
    manager.traffic(1, 147, 141, 146, 147)
    clock[0] += 600.0
    manager.traffic(1, 147, 141, 146, 147)
    assert 'traffic_1' not in manager.completed
