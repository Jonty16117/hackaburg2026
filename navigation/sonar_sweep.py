"""360-degree sonar sweep to discover pool walls at boot time.

The duck rotates in place, logging sonar range vs heading at fixed intervals.
Range minima correspond to perpendicular distances to walls — used to
initialize the WallMap without prior knowledge of pool dimensions.
"""

import math
import time


def median_filter(readings, window=3):
    result = []
    n = len(readings)
    half = window // 2
    for i in range(n):
        vals = []
        for j in range(i - half, i + half + 1):
            r = readings[j % n]
            if r is not None and r > 0:
                vals.append(r)
        if vals:
            vals.sort()
            result.append(vals[len(vals) // 2])
        else:
            result.append(None)
    return result


def _shortest_angle_diff(target, current):
    d = target - current
    return math.atan2(math.sin(d), math.cos(d))


class SonarSweep:
    def __init__(self, drive, sonar, sweep_speed=0.3, step_deg=10,
                 sonar_samples=3, blind_zone_cm=20):
        self.drive = drive
        self.sonar = sonar
        self.sweep_speed = sweep_speed
        self.step_deg = step_deg
        self.sonar_samples = sonar_samples
        self.blind_zone_cm = blind_zone_cm

        self.readings = []
        self.blind_headings = []
        self.start_x = None
        self.start_y = None
        self.duck_pose_x = 0.0
        self.duck_pose_y = 0.0
        self.duck_pose_theta = 0.0

    def run(self, odom):
        self.start_x, self.start_y, _ = odom.position()
        self.duck_pose_x = self.start_x
        self.duck_pose_y = self.start_y
        self.duck_pose_theta = 0.0

        self._sweep_full(odom)

        if len(self.blind_headings) > 0:
            self._escape_blind(odom)
            self._sweep_blind_arcs(odom)

        self.duck_pose_theta = 0.0
        total_rot = self._compute_total_rotation()
        _, _, start_th = odom.x, odom.y, 0.0

        return {
            "readings": self.readings,
            "duck_x": odom.x if hasattr(odom, 'x') else self.start_x,
            "duck_y": odom.y if hasattr(odom, 'y') else self.start_y,
            "total_rotation_deg": total_rot,
        }

    def _sweep_full(self, odom):
        self.readings = []
        self.blind_headings = []

        for deg in range(0, 360, self.step_deg):
            self._rotate_to(deg, odom)
            rng = self._read_sonar_median()
            phi = math.radians(deg)
            self.readings.append((phi, rng))
            if rng is None:
                self.blind_headings.append(deg)

        self._rotate_to(0, odom)

    def _escape_blind(self, odom):
        self.drive.drive_speeds(
            -self.sweep_speed * 0.8,
            -self.sweep_speed * 0.8,
        )
        time.sleep(1.5)
        self.drive.drive_speeds(0.0, 0.0)
        time.sleep(0.5)

    def _sweep_blind_arcs(self, odom):
        _, _, th = odom.position() if hasattr(odom, 'position') else (0, 0, 0)
        for blind_deg in self.blind_headings:
            for offset in range(-20, 21, self.step_deg):
                deg = (blind_deg + offset) % 360
                self._rotate_to(deg, odom)
                rng = self._read_sonar_median()
                phi = math.radians(deg)
                idx = deg // self.step_deg
                if idx < len(self.readings):
                    self.readings[idx] = (phi, rng)

    def _rotate_to(self, target_deg, odom):
        target_rad = math.radians(target_deg)
        tolerance = math.radians(3)

        x, y, th = odom.position() if hasattr(odom, 'position') else (0, 0, 0)
        while True:
            x, y, th = odom.position() if hasattr(odom, 'position') else (0, 0, 0)
            err = _shortest_angle_diff(target_rad, th)
            if abs(err) < tolerance:
                break
            turn_dir = 1.0 if err > 0 else -1.0
            spd = min(self.sweep_speed, abs(err) * 0.3 + 0.1)
            self.drive.drive_speeds(-spd * turn_dir, spd * turn_dir)
            time.sleep(0.05)

        self.drive.drive_speeds(0.0, 0.0)
        time.sleep(0.1)

    def _read_sonar_median(self):
        vals = []
        for _ in range(self.sonar_samples):
            d = self.sonar.distance_cm()
            if d is not None and d > self.blind_zone_cm:
                vals.append(d)
            time.sleep(0.03)
        if len(vals) >= 1:
            vals.sort()
            return vals[len(vals) // 2]
        return None

    def _compute_total_rotation(self):
        return 360


def simulate_sweep(sonar_fn, duck_x=500, duck_y=100, step_deg=10):
    readings = []
    for deg in range(0, 360, step_deg):
        phi = math.radians(deg)
        rng = sonar_fn(phi)
        readings.append((phi, rng))
    filtered = median_filter([r[1] for r in readings], window=3)
    result = []
    for i, (phi, _) in enumerate(readings):
        result.append((phi, filtered[i]))
    return result, {"duck_x": duck_x, "duck_y": duck_y}
