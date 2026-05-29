import math


class Grid:
    def __init__(self, width_cm, height_cm, cell_size, obstacles, duck_radius):
        self.width_cm = width_cm
        self.height_cm = height_cm
        self.cell_size = cell_size
        self.duck_radius = duck_radius
        self.w = _ceil_div(width_cm, cell_size)
        self.h = _ceil_div(height_cm, cell_size)
        self.blocked = [[False] * self.w for _ in range(self.h)]
        if obstacles:
            self._build(obstacles)

    def world_to_grid(self, x, y):
        return (int(x / self.cell_size), int(y / self.cell_size))

    def grid_to_world(self, c, r):
        return ((c + 0.5) * self.cell_size, (r + 0.5) * self.cell_size)

    def is_blocked(self, c, r):
        if not (0 <= c < self.w and 0 <= r < self.h):
            return True
        return self.blocked[r][c]

    def in_bounds(self, c, r):
        return 0 <= c < self.w and 0 <= r < self.h

    def neighbors(self, c, r):
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                if dc == 0 and dr == 0:
                    continue
                nc, nr = c + dc, r + dr
                if self.in_bounds(nc, nr):
                    yield (nc, nr)

    def _build(self, obstacles):
        cell_centers = {}
        for r in range(self.h):
            for c in range(self.w):
                cx, cy = self.grid_to_world(c, r)
                blocked = False
                d2_thresh = None
                for o in obstacles:
                    d2 = (cx - o["x"]) ** 2 + (cy - o["y"]) ** 2
                    thresh = self.duck_radius + o["r"]
                    if d2 < thresh * thresh:
                        blocked = True
                        break
                self.blocked[r][c] = blocked


def _ceil_div(a, b):
    return (a + b - 1) // b
