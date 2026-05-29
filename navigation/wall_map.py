"""Wall mapping: orientation detection and fixed-wall management.

Pool dimensions are known (config.py). The sweep at boot determines
the ORIENTATION offset Δθ. Walls start at REFINING and become LOCKED
as sonar observations confirm them — visible on the dashboard.

Each correction by the EKF refines the corresponding wall's confidence.
"""

import math
from navigation.config import (
    PERIMETER_WIDTH_CM,
    PERIMETER_HEIGHT_CM,
    PERIMETER_CM,
    START_X_CM,
    START_Y_CM,
    MAPPER_MIN_SPAN_DEG,
    EKF_LOCK_COVARIANCE,
    EKF_LOCK_MIN_OBS,
)


class Wall:
    __slots__ = ("A", "B", "C", "normal_angle",
                 "obs_count", "locked", "covariance")

    def __init__(self, A, B, C):
        mag = math.hypot(A, B)
        if mag < 1e-12:
            self.A = 0.0
            self.B = 0.0
            self.C = 0.0
            self.normal_angle = 0.0
        else:
            self.A = A / mag
            self.B = B / mag
            self.C = C / mag
            self.normal_angle = math.atan2(self.B, self.A)
        self.obs_count = 0
        self.locked = False
        self.covariance = 10000.0

    def perpendicular_distance(self, x, y):
        return abs(self.A * x + self.B * y + self.C)

    def signed_distance(self, x, y):
        return self.A * x + self.B * y + self.C

    def is_visible(self, heading, half_cone_deg=37.5):
        diff = heading - self.normal_angle
        diff = math.atan2(math.sin(diff), math.cos(diff))
        return abs(diff) <= math.radians(half_cone_deg)

    def refine(self, x, y, sonar_range):
        delta = abs(self.signed_distance(x, y)) - sonar_range
        nx = x + sonar_range * self.A
        ny = y + sonar_range * self.B
        self.obs_count += 1
        alpha = 1.0 / self.obs_count
        self.C = (1.0 - alpha) * self.C + alpha * (-self.A * nx - self.B * ny)
        self.covariance = max(
            0.1,
            self.covariance * (1.0 - alpha) + alpha * delta * delta,
        )

    def to_dict(self):
        return {
            "A": round(self.A, 6),
            "B": round(self.B, 6),
            "C": round(self.C, 3),
            "normal": round(self.normal_angle, 4),
            "obs_count": self.obs_count,
            "locked": self.locked,
            "covariance": round(self.covariance, 1),
        }


class WallMap:
    def __init__(self):
        self.walls = []
        self.delta_theta = 0.0
        self.n_walls_initially = 0
        self.phase = "INIT"

    def init_known_walls(self, delta_theta=0.0):
        self.delta_theta = delta_theta
        self.walls = [
            Wall(1, 0, -PERIMETER_WIDTH_CM),
            Wall(0, 1, -PERIMETER_HEIGHT_CM),
            Wall(-1, 0, 0),
            Wall(0, -1, 0),
        ]
        self.phase = "REFINING"

    def init_from_sweep(self, readings, duck_x=START_X_CM, duck_y=START_Y_CM):
        segments = self._segment_plateaus(readings)
        delta, n_walls, warning = self._detect_orientation(
            segments, duck_x, duck_y,
        )
        self.init_known_walls(delta_theta=delta)
        self.n_walls_initially = n_walls
        return {
            "delta_theta_deg": round(math.degrees(delta), 1),
            "walls_seen": n_walls,
            "prefix": "WARNING!" if warning else "OK",
            "warning": warning,
        }

    def lock_wall(self, idx, cov_threshold=None, min_obs=None):
        if cov_threshold is None:
            cov_threshold = EKF_LOCK_COVARIANCE
        if min_obs is None:
            min_obs = EKF_LOCK_MIN_OBS
        wall = self.walls[idx]
        if wall.covariance < cov_threshold or wall.obs_count >= min_obs:
            wall.locked = True
        locked_count = sum(1 for w in self.walls if w.locked)
        if locked_count >= 4:
            self.phase = "LOCKED"

    def _segment_plateaus(self, readings):
        n = len(readings)
        valid = [(i, readings[i][0], readings[i][1])
                 for i in range(n) if readings[i][1] is not None]
        if len(valid) < 2:
            return []
        splits = []
        for j in range(len(valid)):
            i1, phi1, r1 = valid[j]
            i2, phi2, r2 = valid[(j + 1) % len(valid)]
            raw_gap = i2 - i1
            if raw_gap < 0:
                raw_gap += n
            if abs(r1 - r2) > 40 or raw_gap > 3:
                splits.append(j + 1)
        if len(splits) <= 1:
            phis = [v[1] for v in valid]
            rngs = [v[2] for v in valid]
            mid = self._circular_mean(phis)
            return [(mid, min(rngs), math.radians(360))]
        segments = []
        for s_idx in range(len(splits)):
            start = splits[s_idx]
            end = splits[(s_idx + 1) % len(splits)]
            if end > start:
                seg_data = valid[start:end]
            else:
                seg_data = valid[start:] + valid[:end]
            if len(seg_data) < 2:
                continue
            phis = [v[1] for v in seg_data]
            rngs = [v[2] for v in seg_data]
            span_deg = len(seg_data) * (360.0 / n)
            if span_deg < MAPPER_MIN_SPAN_DEG:
                continue
            mid = self._circular_mean(phis)
            min_r = min(rngs)
            segments.append((mid, min_r, math.radians(span_deg)))
        segments.sort(key=lambda s: s[2], reverse=True)
        return segments[:4]

    @staticmethod
    def _circular_mean(angles):
        cos_s = sum(math.cos(a) for a in angles)
        sin_s = sum(math.sin(a) for a in angles)
        return math.atan2(sin_s, cos_s)

    def _detect_orientation(self, segments, duck_x, duck_y):
        expected_other_side = abs(PERIMETER_WIDTH_CM - duck_x)
        tolerance = abs(duck_x - expected_other_side)
        if duck_x == expected_other_side:
            tolerance = 120
        delta = 0.0
        warning = None
        n_seen = len(segments)
        if n_seen == 0:
            return 0.0, n_seen, "No wall plateaus detected — using Δθ=0"
        segs = [(seg[0], seg[1]) for seg in segments]
        normal_0, r0 = segs[0]
        far_from_0 = None
        for phi, rng in segs[1:]:
            raw_diff = phi - normal_0
            diff = abs(math.atan2(math.sin(raw_diff), math.cos(raw_diff)))
            if 1.2 < diff < 1.95:
                far_from_0 = (phi, rng)
                break
        if far_from_0 is not None:
            phi_orth, r_orth = far_from_0
            if r0 > r_orth:
                x_candidate = normal_0
            else:
                x_candidate = phi_orth
        elif n_seen >= 2:
            if segs[0][1] > segs[1][1]:
                x_candidate = segs[0][0]
            else:
                x_candidate = segs[1][0]
        else:
            x_candidate = segs[0][0]
        x_best = x_candidate
        for test_phi in [x_candidate, x_candidate + math.pi]:
            test_phi = math.atan2(math.sin(test_phi), math.cos(test_phi))
            if abs(test_phi) < abs(x_best):
                x_best = test_phi
        delta = x_best
        side_dist = expected_other_side
        best_measured = None
        for _, rng in segs:
            if abs(rng - side_dist) < abs((best_measured or 0) - side_dist):
                best_measured = rng
        if best_measured is not None and abs(best_measured - side_dist) > tolerance:
            warning = (
                f"Expected side wall at ~{side_dist:.0f}cm, "
                f"measured {best_measured:.0f}cm — duck may not be at "
                f"({START_X_CM},{START_Y_CM})"
            )
        return delta, n_seen, warning

    def nearest_visible_wall(self, x, y, theta):
        best_idx = None
        best_dist = float("inf")
        for i, wall in enumerate(self.walls):
            if not wall.is_visible(theta):
                continue
            d = wall.perpendicular_distance(x, y)
            if d < best_dist:
                best_dist = d
                best_idx = i
        if best_idx is None:
            return None, None
        return best_idx, best_dist

    def to_perimeter(self):
        from navigation.perimeter import Perimeter
        return Perimeter(PERIMETER_CM)

    def to_dict(self):
        return {
            "walls": [w.to_dict() for w in self.walls],
            "corners": [
                (0, 0),
                (PERIMETER_WIDTH_CM, 0),
                (PERIMETER_WIDTH_CM, PERIMETER_HEIGHT_CM),
                (0, PERIMETER_HEIGHT_CM),
            ],
            "phase": self.phase,
            "walls_locked": sum(1 for w in self.walls if w.locked),
            "delta_theta_deg": round(math.degrees(self.delta_theta), 1),
        }
