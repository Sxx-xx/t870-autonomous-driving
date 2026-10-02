"""검사 창의 감지 결과가 어느 경로로 이어지는지 고정한다.

선택기는 원래 'zone A 선택 = choice 1' 규약이었다. 그런데 검사 창이 어느
칸을 보느냐는 미션마다 다르다. T주차는 창이 1번 칸을 보므로 감지가 곧
'1번 칸이 막혔다' 는 뜻이고, 그러면 2번으로 가야 한다. 그래서 choice 를
zone 라벨이 아니라 choice_when_blocked 에서 뽑는다.
"""
import pytest

from t870_control.competition_mission_manager_node import (
    CompetitionMissionManager)


def resolve_choice(choice_when_blocked, detected):
    """finalize_decision 의 choice 계산과 같은 식."""
    return (choice_when_blocked if detected
            else 3 - choice_when_blocked)


@pytest.mark.parametrize(
    ('detected', 'expected_choice'),
    [(True, 2), (False, 1)])
def test_t_parking_detection_selects_route_two(detected, expected_choice):
    # 대회 launch 의 competition_t_selector 설정값이다.
    assert resolve_choice(2, detected) == expected_choice


@pytest.mark.parametrize(
    ('detected', 'expected_choice'),
    [(True, 2), (False, 1)])
def test_parallel_detection_selects_route_two(detected, expected_choice):
    # 대회 launch 의 competition_parallel_selector 설정값이다.
    assert resolve_choice(2, detected) == expected_choice


@pytest.mark.parametrize(
    ('detected', 'expected_choice'),
    [(True, 1), (False, 2)])
def test_default_mapping_is_unchanged(detected, expected_choice):
    # 단독 주차 launch 는 기본값 1 을 그대로 쓴다.
    assert resolve_choice(1, detected) == expected_choice


def switch_target(t_choice, waypoint=299):
    """미션 매니저가 그 choice 에서 실제로 경로를 바꾸는지 본다."""
    manager = object.__new__(CompetitionMissionManager)
    manager.route = 'T1'
    manager.t_choice = t_choice
    manager.p_choice = 1
    manager.ox_straight = False
    manager.hill_wp = 39
    manager.timed_estop_wp = 532
    manager.stop_duration = 3.0
    manager.completed = {'hill_wp39'}
    manager.traffic_red_seen = {1: True, 2: True, 3: True}
    manager.traffic_green = True
    manager.estop_until = 0.0
    manager.current_waypoint = 0
    switched = []
    manager.switch = lambda route, key, start, reverse='': (
        switched.append(route), setattr(manager, 'route', route))
    manager.timed_stop = lambda key: None
    manager.traffic = lambda *args: None

    class Msg:
        data = waypoint

    manager.on_waypoint(Msg())
    return switched


def test_detection_switches_to_t2():
    # 감지 -> choice 2 -> T_Parking2 로 전환된다.
    assert switch_target(2) == ['T2']


def test_no_detection_keeps_t1():
    # 미감지 -> choice 1 -> 전환 없음, T_Parking1 그대로.
    assert switch_target(1) == []


def parallel_switch_target(p_choice, waypoint=638):
    """P1 을 달리는 중에 그 choice 로 실제 전환이 일어나는지 본다."""
    manager = object.__new__(CompetitionMissionManager)
    manager.route = 'P1'
    manager.t_choice = 1
    manager.p_choice = p_choice
    manager.ox_straight = False
    manager.hill_wp = 39
    manager.timed_estop_wp = 532
    manager.stop_duration = 3.0
    manager.completed = {'hill_wp39', 'estop_wp532'}
    manager.traffic_red_seen = {1: True, 2: True, 3: True}
    manager.traffic_green = True
    manager.estop_until = 0.0
    manager.current_waypoint = 0
    switched = []
    manager.switch = lambda route, key, start, reverse='': (
        switched.append(route), setattr(manager, 'route', route))
    manager.timed_stop = lambda key: None
    manager.traffic = lambda *args: None

    class Msg:
        data = waypoint

    manager.on_waypoint(Msg())
    return switched


def test_parallel_detection_switches_to_p2():
    # 감지 -> choice 2 -> P_Parking2 로 전환된다.
    assert parallel_switch_target(2) == ['P2']


def test_parallel_no_detection_keeps_p1():
    # 미감지 -> choice 1 -> 전환 없음, P_Parking1 그대로.
    assert parallel_switch_target(1) == []


# --- 결정을 한 번만 쏘면 놓쳤을 때 복구가 안 된다 --------------------
# emergency_stop_node 의 finished 와 같은 유형이다. 노드는 답을 알고 있는데
# 그 사실이 밖으로 안 나가면, 미션 매니저는 기본값(choice 1)을 그대로 쓴다.
# 그러면 주차 경로가 통째로 틀린다. ox_signal_detector 는 원래 타이머로
# 계속 쏘고 있었다.

def make_selector(choice_when_blocked=2):
    from t870_control.parking_slot_selector_node import ParkingSlotSelector
    selector = object.__new__(ParkingSlotSelector)
    selector.locked = False
    selector.active = True
    selector.decision = None
    selector.publish_path = False
    selector.blocked_votes = 5
    selector.blocked_votes_required = 5
    selector.scans_seen = 30
    selector.point_counts = []
    selector.choice_when_blocked = choice_when_blocked
    selector.slot_a_path = 'a.csv'
    selector.slot_b_path = 'b.csv'
    selector.results = []
    selector.choices = []
    selector.paths = []
    selector.result_pub = type('P', (), {
        'publish': lambda _s, m: selector.results.append(m.data)})()
    selector.choice_pub = type('P', (), {
        'publish': lambda _s, m: selector.choices.append(m.data)})()
    selector.path_pub = type('P', (), {
        'publish': lambda _s, m: selector.paths.append(m.data)})()
    selector.get_logger = lambda: type(
        'L', (), {'warning': lambda _s, *a: None})()
    return selector


def test_decision_is_republished_after_locking():
    selector = make_selector()
    selector.finalize_decision('test')
    assert selector.choices == [2]
    for _ in range(3):
        selector.republish_decision()
    assert selector.choices == [2, 2, 2, 2], '잠근 뒤에도 계속 알려야 한다'
    assert selector.results == ['B', 'B', 'B', 'B']


def test_republish_does_not_resend_the_path():
    """경로를 되풀이해 보내면 follower 가 최근접 색인을 다시 잡는다."""
    selector = make_selector()
    selector.publish_path = True
    selector.finalize_decision('test')
    assert len(selector.paths) == 1
    for _ in range(5):
        selector.republish_decision()
    assert len(selector.paths) == 1, '경로는 한 번만 보내야 한다'


def test_nothing_is_published_before_a_decision():
    selector = make_selector()
    for _ in range(3):
        selector.republish_decision()
    assert selector.choices == []
    assert selector.results == []


def test_two_selectors_must_not_share_the_result_topic():
    """선택기가 두 개다. 잠근 뒤 계속 재발행하므로 토픽이 같으면 두 값이
    영원히 뒤섞인다. 기능에는 영향이 없지만 현장에서 오판하게 된다."""
    launch = open('src/t870_control/launch/t870_competition.launch.py',
                  encoding='utf-8').read()
    import re
    topics = re.findall(r"'result_topic':\s*'([^']+)'", launch)
    assert len(topics) == 2, '선택기 두 개 모두 result_topic 을 줘야 한다'
    assert len(set(topics)) == 2, '두 선택기가 같은 결과 토픽을 쓴다: %s' % topics
    choices = re.findall(r"'choice_topic':\s*'([^']+)'", launch)
    assert len(set(choices)) == 2, '두 선택기가 같은 choice 토픽을 쓴다'


# --- 결정을 계속 재발행해도 되는가 -------------------------------------
# 선택기가 0.2 초마다 choice 를 다시 쏜다. 그게 경로 추종이나 다음 미션을
# 흔들면 안 된다.

def hammer(route, choice_attr, waypoints, choice_value):
    """choice 를 계속 받으면서 WP 를 진행시킨다. 전환 기록을 돌려준다."""
    manager = object.__new__(CompetitionMissionManager)
    manager.route = route
    manager.t_choice = 1
    manager.p_choice = 1
    manager.ox_straight = False
    manager.hill_wp = -1
    manager.timed_estop_wp = -1
    manager.stop_duration = 3.0
    manager.completed = set()
    manager.traffic_green = True
    manager.estop_until = 0.0
    manager.current_waypoint = 0
    switched = []
    manager.switch = lambda r, key, start, reverse='': (
        switched.append((r, start)), setattr(manager, 'route', r))
    manager.timed_stop = lambda key: None
    manager.traffic = lambda *args: None

    class Msg:
        def __init__(self, data):
            self.data = data

    for waypoint in waypoints:
        # 매 WP 마다 재발행이 여러 번 들어온다고 보고 두들긴다.
        for _ in range(5):
            if choice_attr == 't_choice':
                manager.on_t_choice(Msg(choice_value))
            else:
                manager.on_p_choice(Msg(choice_value))
        manager.on_waypoint(Msg(waypoint))
    return switched


def test_repeated_choice_switches_the_t_path_only_once():
    """재발행을 계속 받아도 경로 전환은 한 번이어야 한다.

    두 번 걸리면 follower 가 최근접 색인을 다시 잡아 추종이 리셋된다.
    """
    switched = hammer('T1', 't_choice', range(299, 330), 2)
    assert switched == [('T2', 299)], switched


def test_repeated_choice_does_not_block_the_next_mission():
    """T2 로 간 뒤 WP335 에서 평행(P1) 으로 넘어가는 것을 막으면 안 된다."""
    switched = hammer('T1', 't_choice', range(299, 345), 2)
    assert switched == [('T2', 299), ('P1', 335)], switched


def test_repeated_parallel_choice_switches_only_once():
    switched = hammer('P1', 'p_choice', range(638, 700), 2)
    assert switched[0] == ('P2', 638)
    assert [r for r, _ in switched].count('P2') == 1, switched


def test_no_detection_never_switches_however_many_times_it_is_sent():
    assert hammer('T1', 't_choice', range(299, 330), 1) == []
