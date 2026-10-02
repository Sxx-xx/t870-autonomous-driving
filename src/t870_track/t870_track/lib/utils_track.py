"""Control algorithms used by the T870 path tracker."""

import math


class PurePursuit:
    """Pure Pursuit steering controller using base_link waypoints."""

    def __init__(self, vehicle_length=1.08):
        self.vehicle_length = vehicle_length
        self.current_vel = 0.0

    def get_vel_status(self, msg):
        """Accept legacy Velocity or geometry_msgs/Twist speed feedback."""
        if hasattr(msg, 'velocity'):
            self.current_vel = msg.velocity
        elif hasattr(msg, 'linear'):
            self.current_vel = msg.linear.x

    def calculate_steering_angle(self, waypoints_msg, lookahead_distance=1.5):
        """Return steering radians and the selected look-ahead waypoint."""
        if not waypoints_msg.waypoints:
            return 0.0, None

        target_point = waypoints_msg.waypoints[-1]
        for waypoint in waypoints_msg.waypoints:
            if math.hypot(waypoint.x, waypoint.y) >= lookahead_distance:
                target_point = waypoint
                break

        distance = math.hypot(target_point.x, target_point.y)
        if distance < 0.1:
            return 0.0, target_point

        alpha = math.atan2(target_point.y, target_point.x)
        steering = math.atan2(
            2.0 * self.vehicle_length * math.sin(alpha), distance)
        return steering, target_point


class PIDController:
    """PID velocity controller with bounded integral wind-up."""

    def __init__(self, p=1.0, i=0.4, d=0.35, dt=0.05):
        self.p_gain = p
        self.i_gain = i
        self.d_gain = d
        self.control_time = dt
        self.prev_error = 0.0
        self.i_control = 0.0

    def calculate(self, current_vel, target_velocity):
        """Calculate the bounded-integral PID output."""
        error = target_velocity - current_vel
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.control_time
        self.i_control = max(-1.0, min(1.0, self.i_control))
        d_control = self.d_gain * (
            error - self.prev_error) / self.control_time
        self.prev_error = error
        return p_control + self.i_control + d_control

    def reset(self):
        """Clear accumulated state whenever tracking is stopped."""
        self.prev_error = 0.0
        self.i_control = 0.0
