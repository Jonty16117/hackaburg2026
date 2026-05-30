"""360-degree sonar sweep to discover pool orientation at boot time.

The duck rotates in place using timed motor steps, logging sonar range
vs body-frame heading at fixed intervals. The sweep determines the
rotation offset Δθ between the duck's body heading and pool +x.
"""

import math
import time

from navigation.config import (
    BRAIN_CFG,
    MAPPER_SWEEP_SPEED,
    MAPPER_SWEEP_DEG_STEP,
    MAPPER_SONAR_SAMPLES,
    MAPPER_BLIND_ZONE_CM,
    MAPPER_BLIND_REVERSE_CM,
)

MAX_SPEED_CM_S = BRAIN_CFG["MAX_SPEED_CM_S"]
WHEEL_BASE_CM = BRAIN_CFG["WHEEL_BASE_CM"]


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


class SonarSweep:
    def __init__(self, drive, sonar, sweep_speed=None, step_deg=None,
                 sonar_samples=None, blind_zone_cm=None):
        self.drive = drive
        self.sonar = sonar
        self.sweep_speed = sweep_speed if sweep_speed is not None else MAPPER_SWEEP_SPEED
        self.step_deg = step_deg if step_deg is not None else MAPPER_SWEEP_DEG_STEP
        self.sonar_samples = sonar_samples if sonar_samples is not None else MAPPER_SONAR_SAMPLES
        self.blind_zone_cm = blind_zone_cm if blind_zone_cm is not None else MAPPER_BLIND_ZONE_CM

        self.readings = []
        self.blind_headings = []
        self._current_heading_deg = 0.0

    def run(self, odom):
        self._sweep_full(odom)

        if self.blind_headings:
            self._escape_blind_from_headings(odom)
            self._sweep_full(odom, only_blind=True)

        self._return_to_start(odom)

        return {
            "readings": self.readings,
            "duck_x": odom.x,
            "duck_y": odom.y,
        }

    def _angular_speed_rad_s(self):
        return (self.sweep_speed * MAX_SPEED_CM_S * 2.0) / WHEEL_BASE_CM

    def _rotation_time_ms(self):
        rad_per_step = math.radians(self.step_deg)
        angular_spd = self._angular_speed_rad_s()
        if angular_spd < 1e-9:
            return 200
        return max(80, int((rad_per_step / angular_spd) * 1000 * 1.15))

    def _sweep_full(self, odom, only_blind=False):
        if not only_blind:
            self.readings = []
            self.blind_headings = []

        rot_ms = self._rotation_time_ms()

        for deg in range(0, 360, self.step_deg):
            if only_blind:
                near_blind = any(
                    abs((deg - bd + 180) % 360 - 180) < 30
                    for bd in self.blind_headings
                )
                if not near_blind:
                    continue

            self._rotate_step(rot_ms, odom)
            rng = self._read_sonar_median()
            phi = math.radians(deg)
            self._current_heading_deg = deg

            if only_blind:
                idx = deg // self.step_deg
                if idx < len(self.readings):
                    self.readings[idx] = (phi, rng)
            else:
                self.readings.append((phi, rng))
                if rng is None:
                    self.blind_headings.append(deg)

    def _rotate_step(self, duration_ms, odom):
        turn_dir = 1
        left = -self.sweep_speed * turn_dir
        right = self.sweep_speed * turn_dir

        self.drive.drive_speeds(left, right)
        t0 = time.time()
        elapsed = 0.0
        while elapsed < (duration_ms / 1000.0):
            time.sleep(0.01)
            elapsed = time.time() - t0

        self.drive.drive_speeds(0.0, 0.0)
        time.sleep(0.08)

        dt = elapsed
        odom.update(left, right, dt)

    def _escape_blind_from_headings(self, odom):
        time.sleep(0.5)

    def _return_to_start(self, odom):
        rem = (360 - self._current_heading_deg) % 360
        if rem < 3:
            return
        rot_ms = int((rem / self.step_deg) * self._rotation_time_ms())
        self._rotate_step(rot_ms, odom)

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
