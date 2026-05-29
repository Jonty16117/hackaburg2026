"""Perimeter awareness: point-in-polygon, distance-to-edge, bearing-to-center."""

import math


class Perimeter:
    def __init__(self, vertices_cm):
        self.vertices = vertices_cm
        self._compute_bounds()
        self._compute_centroid()

    def _compute_bounds(self):
        xs = [v[0] for v in self.vertices]
        ys = [v[1] for v in self.vertices]
        self.min_x = min(xs)
        self.max_x = max(xs)
        self.min_y = min(ys)
        self.max_y = max(ys)

    def _compute_centroid(self):
        n = len(self.vertices)
        cx = sum(v[0] for v in self.vertices) / n
        cy = sum(v[1] for v in self.vertices) / n
        self.centroid = (cx, cy)

    def is_inside(self, x, y):
        n = len(self.vertices)
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = self.vertices[i]
            xj, yj = self.vertices[j]
            if ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / (yj - yi) + xi
            ):
                inside = not inside
            j = i
        return inside

    def distance_to_edge(self, x, y):
        n = len(self.vertices)
        min_dist = float("inf")
        for i in range(n):
            x1, y1 = self.vertices[i]
            x2, y2 = self.vertices[(i + 1) % n]
            dx = x2 - x1
            dy = y2 - y1
            length_sq = dx * dx + dy * dy
            if length_sq == 0:
                dist = math.hypot(x - x1, y - y1)
            else:
                t = max(
                    0.0,
                    min(1.0, ((x - x1) * dx + (y - y1) * dy) / length_sq),
                )
                px = x1 + t * dx
                py = y1 + t * dy
                dist = math.hypot(x - px, y - py)
            min_dist = min(min_dist, dist)
        return min_dist

    def is_near_edge(self, x, y, margin):
        return self.distance_to_edge(x, y) < margin

    def bearing_to_center(self, x, y):
        dx = self.centroid[0] - x
        dy = self.centroid[1] - y
        return math.atan2(dy, dx)


