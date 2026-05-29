"""Waypoint tracking with proportional heading control."""

import math
from navigation.utils import heading_error, clamp


class PathFollower:
    def __init__(self, path, max_speed, heading_tolerance, arrival_dist=20):
        self.path = path
        self.max_speed = max_speed
        self.heading_tolerance = heading_tolerance
        self.arrival_dist = arrival_dist
        self.waypoint_index = 0
        self.arrived = False

    def compute_speeds(self, x, y, theta):
        if self.arrived or not self.path or self.waypoint_index >= len(self.path):
            self.arrived = True
            return 0.0, 0.0

        wx, wy = self.path[self.waypoint_index]
        dist = math.hypot(wx - x, wy - y)

        if dist < self.arrival_dist:
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.path):
                self.arrived = True
                return 0.0, 0.0
            wx, wy = self.path[self.waypoint_index]

        herr = heading_error(math.atan2(wy - y, wx - x), theta)
        if abs(herr) < self.heading_tolerance:
            return self.max_speed, self.max_speed

        turn_dir = math.copysign(1, herr)
        turn_speed = self.max_speed if abs(herr) > 0.3 else self.max_speed * (abs(herr) / 0.3)

        ls = -turn_speed * turn_dir
        rs = turn_speed * turn_dir

        if abs(herr) < 1.0:
            forward = self.max_speed * (1 - abs(herr))
            ls += forward
            rs += forward

        ls = clamp(ls, -self.max_speed, self.max_speed)
        rs = clamp(rs, -self.max_speed, self.max_speed)
        return ls, rs
