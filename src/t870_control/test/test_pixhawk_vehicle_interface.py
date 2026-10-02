import pytest

from t870_control.control_utils import mps_to_kmh_command, normalized_to_pwm


def test_normalized_to_pwm_center_and_limits():
    assert normalized_to_pwm(-1.0, 1000, 1500, 2000) == 1000
    assert normalized_to_pwm(0.0, 1000, 1500, 2000) == 1500
    assert normalized_to_pwm(1.0, 1000, 1500, 2000) == 2000


def test_normalized_to_pwm_asymmetric_range_and_clamping():
    assert normalized_to_pwm(-0.5, 1100, 1450, 1900) == 1275
    assert normalized_to_pwm(0.5, 1100, 1450, 1900) == 1675
    assert normalized_to_pwm(-2.0, 1100, 1450, 1900) == 1100
    assert normalized_to_pwm(2.0, 1100, 1450, 1900) == 1900


def test_mps_to_kmh_command_and_clamping():
    assert mps_to_kmh_command(0.2, 2.0) == pytest.approx(0.72)
    assert mps_to_kmh_command(-0.2, 2.0) == pytest.approx(-0.72)
    assert mps_to_kmh_command(1.0, 2.0) == 2.0
    assert mps_to_kmh_command(-1.0, 2.0) == -2.0
