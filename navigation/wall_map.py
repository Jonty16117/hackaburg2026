"""Wall mapping: discovers, stores, and manages pool wall estimates.

Wall lifecycle:
  INIT → sweep completes → REFINING (walls discovered, cov high)
         → EKF refines walls → LOCKED (σ² < threshold OR obs >= min)

Each wall is stored as a normalized line equation: A·x + B·y + C = 0
where √(A²+B²) = 1 and C is the signed distance from origin.
"""

import math


class Wall:
    __slots__ = ("A", "B", "C", "normal_angle", "obs_count",
                 "locked", "covariance", "first_point")

    def __init__(self, A, B, C, normal_angle=None):
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
            self.normal_angle = (
                normal_angle if normal_angle is not None
                else math.atan2(self.B, self.A)
            )
        self.obs_count = 0
        self.locked = False
        self.covariance = 10000.0
        self.first_point = None

    def perpendicular_distance(self, x, y):
        d = self.A * x + self.B * y + self.C
        return abs(d)

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
        cs = self.obs_count
        alpha = 1.0 / (cs + 1) if cs > 0 else 1.0
        self.obs_count += 1
        self.C = (1.0 - alpha) * self.C + alpha * (-self.A * nx - self.B * ny)
        if self.first_point is None:
            self.first_point = (nx, ny)
        self.covariance = max(
            1.0,
            self.covariance * (1.0 - alpha) + alpha * delta * delta,
        )

    def to_line_points(self, x_min, y_min, x_max, y_max):
        if abs(self.B) > 1e-9:
            y0 = y_min
            x0 = -(self.B * y0 + self.C) / self.A
            y1 = y_max
            x1 = -(self.B * y1 + self.C) / self.A
        else:
            x0 = -(self.C) / self.A
            y0 = y_min
            x1 = x0
            y1 = y_max
        return (x0, y0, x1, y1)

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
        self.corners = []
        self.phase = "INIT"
        self._bounds = (0.0, 0.0, 1000.0, 200.0)

    def init_from_minima(self, readings, duck_x=0.0, duck_y=0.0):
        segments = self._segment_by_range(readings)
        candidates = []
        for seg_phi, seg_rng, span in segments:
            if span >= math.radians(15):
                candidates.append((seg_phi, seg_rng, span))
        candidates.sort(key=lambda x: x[2], reverse=True)
        candidates = candidates[:4]

        self.walls = []
        for phi_est, r_est, _span in candidates:
            nx = duck_x + r_est * math.cos(phi_est)
            ny = duck_y + r_est * math.sin(phi_est)
            A = math.cos(phi_est)
            B = math.sin(phi_est)
            C = -(A * nx + B * ny)
            wall = Wall(A, B, C, normal_angle=phi_est)
            wall.first_point = (nx, ny)
            wall.obs_count = 1
            self.walls.append(wall)

        self._sort_walls()
        self._compute_corners()
        self.phase = "REFINING"

    def _sort_walls(self):
        self.walls.sort(key=lambda w: math.atan2(
            math.sin(w.normal_angle), math.cos(w.normal_angle)
        ))

    def _segment_by_range(self, readings):
        n = len(readings)
        valid_indices = []
        for i in range(n):
            if readings[i][1] is not None:
                valid_indices.append(i)
        if len(valid_indices) < 2:
            return []

        splits = []
        for j in range(len(valid_indices)):
            i1 = valid_indices[j]
            i2 = valid_indices[(j + 1) % len(valid_indices)]
            d1 = readings[i1][1]
            d2 = readings[i2][1]
            raw_gap = i2 - i1
            if raw_gap < 0:
                raw_gap += n
            if abs(d1 - d2) > 40 or raw_gap > 3:
                splits.append(j + 1)

        if len(splits) <= 1:
            phis = [readings[i][0] for i in valid_indices]
            rngs = [readings[i][1] for i in valid_indices]
            mid = self._circular_midpoint(phis)
            return [(mid, min(rngs), math.radians(360))]

        segments = []
        for s_idx in range(len(splits)):
            start_split = splits[s_idx]
            end_split = splits[(s_idx + 1) % len(splits)]
            if end_split > start_split:
                seg_indices = valid_indices[start_split:end_split]
            else:
                seg_indices = (valid_indices[start_split:] +
                              valid_indices[:end_split])
            if len(seg_indices) < 2:
                continue

            phis = [readings[i][0] for i in seg_indices]
            rngs = [readings[i][1] for i in seg_indices]

            mid_phi = self._circular_midpoint(phis)
            min_rng = min(rngs)

            arc_deg = (len(seg_indices) * (360.0 / n))
            span_rad = max(math.radians(arc_deg), math.radians(10))

            if min_rng > 20:
                segments.append((mid_phi, min_rng, span_rad))

        return segments

    @staticmethod
    def _circular_midpoint(angles):
        cos_sum = 0.0
        sin_sum = 0.0
        for a in angles:
            cos_sum += math.cos(a)
            sin_sum += math.sin(a)
        return math.atan2(sin_sum, cos_sum)

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

    def lock_wall(self, idx, cov_threshold=25.0, min_obs=3):
        wall = self.walls[idx]
        if wall.covariance < cov_threshold or wall.obs_count >= min_obs:
            wall.locked = True
        all_locked = all(w.locked for w in self.walls)
        if all_locked and len(self.walls) >= 4:
            self.phase = "LOCKED"
        return wall.locked

    def _compute_corners(self):
        self.corners = []
        n = len(self.walls)
        if n < 2:
            return
        for i in range(n):
            w1 = self.walls[i]
            w2 = self.walls[(i + 1) % n]
            det = w1.A * w2.B - w2.A * w1.B
            if abs(det) < 1e-12:
                continue
            cx = (w2.B * (-w1.C) - w1.B * (-w2.C)) / det
            cy = (w1.A * (-w2.C) - w2.A * (-w1.C)) / det
            self.corners.append((cx, cy))

    def to_perimeter(self):
        from navigation.perimeter import Perimeter
        if len(self.corners) >= 3:
            return Perimeter(list(self.corners))
        if len(self.walls) >= 3:
            self._compute_bounds_from_walls()
            return Perimeter(self._estimate_vertices())
        if len(self.walls) >= 2:
            self._compute_bounds_from_walls()
            return Perimeter(self._estimate_vertices())
        return Perimeter([(0, 0), (1000, 0), (1000, 200), (0, 200)])

    def _compute_bounds_from_walls(self):
        xs = []
        ys = []
        for w in self.walls:
            points = w.to_line_points(0, 0, 10000, 10000)
            xs.extend([points[0], points[2]])
            ys.extend([points[1], points[3]])
        if xs and ys:
            self._bounds = (min(xs) - 50, min(ys) - 50,
                           max(xs) + 50, max(ys) + 50)

    def _estimate_vertices(self):
        x_min, y_min, x_max, y_max = self._bounds
        return [(x_min, y_min), (x_max, y_min),
                (x_max, y_max), (x_min, y_max)]

    def add_wall(self, normal_angle, perpendicular_distance, duck_pose):
        nx = duck_pose[0] + perpendicular_distance * math.cos(normal_angle)
        ny = duck_pose[1] + perpendicular_distance * math.sin(normal_angle)
        A = math.cos(normal_angle)
        B = math.sin(normal_angle)
        C = -(A * nx + B * ny)
        wall = Wall(A, B, C, normal_angle=normal_angle)
        wall.first_point = (nx, ny)
        wall.obs_count = 1
        self.walls.append(wall)
        self._sort_walls()
        self._compute_corners()
        return len(self.walls) - 1

    def to_dict(self):
        return {
            "walls": [w.to_dict() for w in self.walls],
            "corners": [
                (round(c[0], 1), round(c[1], 1)) for c in self.corners
            ],
            "phase": self.phase,
            "walls_locked": sum(1 for w in self.walls if w.locked),
        }
