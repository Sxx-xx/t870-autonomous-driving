"""Pure helpers for T870 command conversion."""


def normalized_to_pwm(value, minimum, trim, maximum):
    value = max(-1.0, min(1.0, value))
    if value >= 0.0:
        return round(trim + value * (maximum - trim))
    return round(trim + value * (trim - minimum))


def mps_to_kmh_command(speed_mps, max_speed_kmh):
    """Convert a ROS speed command to the Arduino's signed km/h command."""
    speed_kmh = float(speed_mps) * 3.6
    return max(-max_speed_kmh, min(max_speed_kmh, speed_kmh))
