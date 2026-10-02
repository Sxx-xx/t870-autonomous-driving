import pytest

from t870_control.obstacle_avoidance_node import blended_command


def test_no_lidar_correction_preserves_gps_curve():
    command = blended_command(2.2222, -0.08, 0.0, 0.56, 0.20)
    assert command.linear.x == pytest.approx(0.56)
    assert command.angular.z == pytest.approx(-0.08)


def test_lidar_correction_is_added_to_gps_steering():
    command = blended_command(0.56, -0.08, 0.12, 0.56, 0.20)
    assert command.angular.z == pytest.approx(0.04)


def test_blended_steering_is_clamped():
    command = blended_command(0.56, 0.16, 0.12, 0.56, 0.20)
    assert command.angular.z == pytest.approx(0.20)
