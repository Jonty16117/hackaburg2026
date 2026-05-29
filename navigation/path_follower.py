import math


def _normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def _heading_error(target, current):
    e = target - current
    return ((e + math.pi) % (2 * math.pi)) - math.pi


class PathFollower:
    def __init__(self, path, max_speed, heading_tolerance, arrival_dist=20):
        self.path = path
        self.max_speed = max_speed
        self.heading_tolerance = heading_tolerance
        self.arrival_dist = arrival_dist
        self.waypoint_index = 0
        self.arrived = False

    def reset(self, new_path=None):
        if new_path is not None:
            self.path = new_path
        self.waypoint_index = 0
        self.arrived = False

    def compute_speeds(self, x, y, theta):
        if not self.path:
            self.arrived = True
            return (0.0, 0.0)
        if self.arrived:
            return (0.0, 0.0)

        if self.waypoint_index >= len(self.path):
            self.arrived = True
            return (0.0, 0.0)

        # Check if current waypoint is reached
        wx, wy = self.path[self.waypoint_index]
        dist = math.hypot(wx - x, wy - y)

        if dist < self.arrival_dist:
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.path):
                self.arrived = True
                return (0.0, 0.0)
            wx, wy = self.path[self.waypoint_index]

        # Heading to waypoint
        target_th = math.atan2(wy - y, wx - x)
        herr = _heading_error(target_th, theta)

        if abs(herr) < self.heading_tolerance:
            return (self.max_speed, self.max_speed)

        # Proportional turning
        turn_dir = math.copysign(1, herr)
        if abs(herr) > 0.3:
            turn_speed = self.max_speed
        else:
            ratio = abs(herr) / 0.3
            turn_speed = self.max_speed * ratio

        ls = -turn_speed * turn_dir
        rs = turn_speed * turn_dir

        # Mix forward with turn when heading error is moderate
        if abs(herr) < 1.0:
            forward = self.max_speed * (1 - abs(herr))
            ls += forward
            rs += forward

        # Clamp
        ls = _clamp(ls, -self.max_speed, self.max_speed)
        rs = _clamp(rs, -self.max_speed, self.max_speed)

        return (ls, rs)


def _clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v
