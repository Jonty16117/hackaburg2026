"""Dead reckoning: integrates motor speed commands into pose estimate."""

import math
from navigation.utils import normalize_angle


class Odometry:
    def __init__(self, x_cm=0.0, y_cm=0.0, theta_rad=0.0,
                 wheel_base_cm=30.0, max_speed_cm_s=100.0):
        self.x = x_cm
        self.y = y_cm
        self.theta = theta_rad
        self.wheel_base = wheel_base_cm
        self.max_speed = max_speed_cm_s

    def update(self, left_speed, right_speed, dt):
        vl = left_speed * self.max_speed
        vr = right_speed * self.max_speed
        v = (vl + vr) / 2.0
        omega = (vr - vl) / self.wheel_base

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        self.theta = normalize_angle(self.theta)

    def position(self):
        return self.x, self.y, self.theta
