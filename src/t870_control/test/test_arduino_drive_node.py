"""arduino_drive_node 의 정지 종류 구분 검증.

이 차에는 브레이크가 없다. 펌웨어에서 PWM 0 은 관성주행(coast)이라
오르막에서는 그냥 뒤로 밀린다. 그래서 정지를 두 종류로 나눈다.

  안전 E-stop / 방향전환 E-stop -> 'E'  구동 PWM 즉시 차단
  대회 규정 정차(WP39, WP532)   -> 'H'  밀림을 되받는 홀드

안전 쪽이 걸려 있으면 홀드는 절대 나가지 않아야 한다.
"""
from t870_control.arduino_drive_node import ArduinoDriveNode


class FakeSerial:
    def __init__(self):
        self.written = b''
        self.in_waiting = 0

    def write(self, payload):
        self.written += payload

    def read(self, count):
        return b''


def make_node(**overrides):
    node = object.__new__(ArduinoDriveNode)
    node.estop_timeout = 0.5
    node.command_timeout = 0.5
    node.mission_hold_enabled = True
    node.target_kmh = 2.0
    node.estop = False
    node.shift_estop = False
    node.mission_estop = False
    node.dynamic_estop = False
    node.manual_escape_allowed = False
    # 모든 신호를 '방금 들어온 것' 으로 둔다. fresh() 를 시계 없이
    # 판정하려고 now_ns/fresh 를 테스트용으로 갈아끼운다.
    node.last_cmd_ns = 1
    node.last_estop_ns = 1
    node.last_shift_estop_ns = 1
    node.last_mission_estop_ns = 1
    node.last_dynamic_estop_ns = 1
    node.last_manual_escape_ns = 1
    node.rc_mode = 'AUTO'
    node.last_sent = None
    node.serial = FakeSerial()
    node.fresh = lambda stamp_ns, timeout: bool(stamp_ns)
    for name, value in overrides.items():
        setattr(node, name, value)
    return node


def test_mission_stop_requests_hold_not_pwm_cut():
    node = make_node(mission_estop=True)
    assert node.emergency_stop_required() is False
    assert node.mission_hold_required() is True
    assert node.safe_command() == 0.0


def test_safety_estop_wins_over_mission_hold():
    node = make_node(mission_estop=True, estop=True)
    assert node.emergency_stop_required() is True
    assert node.mission_hold_required() is False


def test_shift_estop_wins_over_mission_hold():
    node = make_node(mission_estop=True, shift_estop=True)
    assert node.emergency_stop_required() is True
    assert node.mission_hold_required() is False


def test_disabled_mission_hold_falls_back_to_pwm_cut():
    node = make_node(mission_estop=True, mission_hold_enabled=False)
    assert node.emergency_stop_required() is True
    assert node.mission_hold_required() is False


def test_no_stop_requests_neither():
    node = make_node()
    assert node.emergency_stop_required() is False
    assert node.mission_hold_required() is False
    assert node.safe_command() == 2.0


def test_hold_writes_zero_before_h_for_old_firmware():
    # 'H' 를 모르는 펌웨어는 "? unknown cmd" 를 찍고 직전 목표 속도를
    # 유지한다. 0 을 먼저 보내지 않으면 규정 정차에서 차가 계속 간다.
    node = make_node()
    node.write_command(0.0, emergency_stop=False, mission_hold=True)
    assert node.serial.written == b'0.000\nH\n'


def test_emergency_stop_writes_e_only():
    node = make_node()
    node.write_command(0.0, emergency_stop=True, mission_hold=True)
    assert node.serial.written == b'E\n'


def test_normal_command_writes_speed():
    node = make_node()
    node.write_command(1.25, emergency_stop=False, mission_hold=False)
    assert node.serial.written == b'1.250\n'


def test_dynamic_obstacle_cuts_pwm_immediately():
    # 평범한 0 명령은 펌웨어 DECEL_RATE 램프(0.7 km/h per sec)를 타서
    # 8 km/h 에서 11 초, 약 12 m 를 더 간다. 3 m 앞 장애물에는 못 쓴다.
    # 동적장애물은 E 로 나가 PWM 을 즉시 끊어야 한다.
    node = make_node(dynamic_estop=True)
    assert node.emergency_stop_required() is True
    assert node.mission_hold_required() is False
    assert node.safe_command() == 0.0
    node.write_command(0.0, emergency_stop=True)
    assert node.serial.written == b'E\n'


def test_dynamic_obstacle_inactive_drives_normally():
    node = make_node()
    assert node.emergency_stop_required() is False
    assert node.safe_command() == 2.0
