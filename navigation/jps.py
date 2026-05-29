import math
import heapq

# Direction indices for JPS+ precomputation
_E = 0; _NE = 1; _N = 2; _NW = 3
_W = 4; _SW = 5; _S = 6; _SE = 7

DIRS = [(1, 0), (1, 1), (0, 1), (-1, 1),
        (-1, 0), (-1, -1), (0, -1), (1, -1)]

_DIR_IDX = {d: i for i, d in enumerate(DIRS)}


def _sign(x):
    if x > 0:
        return 1
    if x < 0:
        return -1
    return 0


def _dir_index(dc, dr):
    return _DIR_IDX.get((_sign(dc), _sign(dr)), 0)


# ---------------------------------------------------------------------------
# Vanilla JPS
# ---------------------------------------------------------------------------

def jps_search(grid, start_xy, goal_xy):
    sc, sr = grid.world_to_grid(*start_xy)
    gc, gr = grid.world_to_grid(*goal_xy)

    if grid.is_blocked(sc, sr) or grid.is_blocked(gc, gr):
        return []

    came_from = {}
    g_score = {(sc, sr): 0.0}
    open_heap = [(0.0, sc, sr)]
    open_set = {(sc, sr)}

    while open_heap:
        _, c, r = heapq.heappop(open_heap)
        open_set.discard((c, r))

        if (c, r) == (gc, gr):
            return _reconstruct_path(came_from, c, r, grid)

        parent = came_from.get((c, r))
        successors = _identify_successors(grid, c, r, parent, gc, gr)

        for nc, nr in successors:
            dc = nc - c
            dr = nr - r
            step_cost = (dc * dc + dr * dr) ** 0.5
            tentative_g = g_score[(c, r)] + step_cost

            key = (nc, nr)
            if key not in g_score or tentative_g < g_score[key]:
                came_from[key] = (c, r)
                g_score[key] = tentative_g
                h = ((gc - nc) ** 2 + (gr - nr) ** 2) ** 0.5
                f = tentative_g + h
                if key not in open_set:
                    heapq.heappush(open_heap, (f, nc, nr))
                    open_set.add(key)

    return []


def _identify_successors(grid, c, r, parent, gc, gr):
    successors = []
    for dc, dr in _get_directions(c, r, parent, grid):
        jp = _jump(grid, c, r, dc, dr, gc, gr)
        if jp is not None:
            successors.append(jp)
    return successors


def _get_directions(c, r, parent, grid):
    if parent is None:
        return DIRS[:]

    pc, pr = parent
    dc = _sign(c - pc)
    dr = _sign(r - pr)

    dirs = []
    if dc != 0 and dr != 0:
        dirs.append((dc, dr))
        dirs.append((dc, 0))
        dirs.append((0, dr))
        if grid.is_blocked(c - dc, r):
            dirs.append((-dc, 0))
            dirs.append((-dc, dr))
        if grid.is_blocked(c, r - dr):
            dirs.append((0, -dr))
            dirs.append((dc, -dr))
    elif dc != 0:
        dirs.append((dc, 0))
        dirs.append((dc, 1))
        dirs.append((dc, -1))
        if grid.is_blocked(c, r + 1):
            dirs.append((0, 1))
            dirs.append((dc, 1))
        if grid.is_blocked(c, r - 1):
            dirs.append((0, -1))
            dirs.append((dc, -1))
    else:
        dirs.append((0, dr))
        dirs.append((1, dr))
        dirs.append((-1, dr))
        if grid.is_blocked(c + 1, r):
            dirs.append((1, 0))
            dirs.append((1, dr))
        if grid.is_blocked(c - 1, r):
            dirs.append((-1, 0))
            dirs.append((-1, dr))

    return _unique_dirs(dirs)


def _unique_dirs(dirs):
    seen = set()
    result = []
    for d in dirs:
        if d not in seen:
            seen.add(d)
            result.append(d)
    return result


def _jump(grid, c, r, dc, dr, gc, gr):
    nc, nr = c + dc, r + dr
    if not grid.in_bounds(nc, nr) or grid.is_blocked(nc, nr):
        return None
    if nc == gc and nr == gr:
        return (nc, nr)

    if dc != 0 and dr != 0:
        if grid.is_blocked(nc - dc, nr) and not grid.is_blocked(nc - dc, nr + dr):
            return (nc, nr)
        if grid.is_blocked(nc, nr - dr) and not grid.is_blocked(nc + dc, nr - dr):
            return (nc, nr)
        if (_jump(grid, nc, nr, dc, 0, gc, gr) is not None or
                _jump(grid, nc, nr, 0, dr, gc, gr) is not None):
            return (nc, nr)
    else:
        if _has_forced(grid, nc, nr, dc, dr):
            return (nc, nr)

    return _jump(grid, nc, nr, dc, dr, gc, gr)


def _has_forced(grid, c, r, dc, dr):
    if dr == 0:
        above = grid.is_blocked(c, r + 1) and not grid.is_blocked(c + dc, r + 1)
        below = grid.is_blocked(c, r - 1) and not grid.is_blocked(c + dc, r - 1)
        return above or below
    else:
        left = grid.is_blocked(c - 1, r) and not grid.is_blocked(c - 1, r + dr)
        right = grid.is_blocked(c + 1, r) and not grid.is_blocked(c + 1, r + dr)
        return left or right


def _reconstruct_path(came_from, c, r, grid):
    path = []
    key = (c, r)
    while key in came_from:
        wx, wy = grid.grid_to_world(*key)
        path.append((wx, wy))
        key = came_from[key]
    wx, wy = grid.grid_to_world(*key)
    path.append((wx, wy))
    path.reverse()
    return path


# ---------------------------------------------------------------------------
# JPS+ (precomputed jump distances)
# ---------------------------------------------------------------------------

class JPSPlusGrid:
    def __init__(self, width_cm, height_cm, cell_size, obstacles, duck_radius):
        from navigation.grid import Grid
        self._grid = Grid(width_cm, height_cm, cell_size, obstacles, duck_radius)
        self.w = self._grid.w
        self.h = self._grid.h
        self.cell_size = cell_size
        self._jump_dist = None
        self._dirty = True

    def world_to_grid(self, x, y):
        return self._grid.world_to_grid(x, y)

    def grid_to_world(self, c, r):
        return self._grid.grid_to_world(c, r)

    def in_bounds(self, c, r):
        return self._grid.in_bounds(c, r)

    def is_blocked(self, c, r):
        return self._grid.is_blocked(c, r)

    def precompute(self):
        w, h = self.w, self.h
        self._jump_dist = [[[0] * 8 for _ in range(w)] for _ in range(h)]
        b = self._grid.blocked

        for r in range(h):
            dist = 0
            for c in range(w - 1, -1, -1):
                dist = 0 if b[r][c] else dist + 1
                self._jump_dist[r][c][_E] = dist
        for r in range(h):
            dist = 0
            for c in range(w):
                dist = 0 if b[r][c] else dist + 1
                self._jump_dist[r][c][_W] = dist
        for c in range(w):
            dist = 0
            for r in range(h - 1, -1, -1):
                dist = 0 if b[r][c] else dist + 1
                self._jump_dist[r][c][_N] = dist
        for c in range(w):
            dist = 0
            for r in range(h):
                dist = 0 if b[r][c] else dist + 1
                self._jump_dist[r][c][_S] = dist

        for r in range(h - 1, -1, -1):
            for c in range(w - 1, -1, -1):
                if b[r][c]:
                    self._jump_dist[r][c][_NE] = 0
                elif r + 1 >= h or c + 1 >= w:
                    self._jump_dist[r][c][_NE] = 1
                else:
                    self._jump_dist[r][c][_NE] = 1 + min(
                        self._jump_dist[r + 1][c + 1][_E],
                        self._jump_dist[r + 1][c + 1][_N],
                        self._jump_dist[r + 1][c + 1][_NE],
                    )

        for r in range(h - 1, -1, -1):
            for c in range(w):
                if b[r][c]:
                    self._jump_dist[r][c][_NW] = 0
                elif r + 1 >= h or c - 1 < 0:
                    self._jump_dist[r][c][_NW] = 1
                else:
                    self._jump_dist[r][c][_NW] = 1 + min(
                        self._jump_dist[r + 1][c - 1][_W],
                        self._jump_dist[r + 1][c - 1][_N],
                        self._jump_dist[r + 1][c - 1][_NW],
                    )

        for r in range(h):
            for c in range(w):
                if b[r][c]:
                    self._jump_dist[r][c][_SW] = 0
                elif r - 1 < 0 or c - 1 < 0:
                    self._jump_dist[r][c][_SW] = 1
                else:
                    self._jump_dist[r][c][_SW] = 1 + min(
                        self._jump_dist[r - 1][c - 1][_W],
                        self._jump_dist[r - 1][c - 1][_S],
                        self._jump_dist[r - 1][c - 1][_SW],
                    )

        for r in range(h):
            for c in range(w - 1, -1, -1):
                if b[r][c]:
                    self._jump_dist[r][c][_SE] = 0
                elif r - 1 < 0 or c + 1 >= w:
                    self._jump_dist[r][c][_SE] = 1
                else:
                    self._jump_dist[r][c][_SE] = 1 + min(
                        self._jump_dist[r - 1][c + 1][_E],
                        self._jump_dist[r - 1][c + 1][_S],
                        self._jump_dist[r - 1][c + 1][_SE],
                    )

        self._dirty = False

    def jump_dist(self, c, r, dc, dr):
        di = _dir_index(dc, dr)
        return self._jump_dist[r][c][di]


def _jump_jpsplus(jpsp, c, r, dc, dr, gc, gr):
    if dc != 0 and dr != 0:
        return _jump_diag_jpsplus(jpsp, c, r, dc, dr, gc, gr)
    else:
        return _jump_cardinal_jpsplus(jpsp, c, r, dc, dr, gc, gr)


def _jump_cardinal_jpsplus(jpsp, c, r, dc, dr, gc, gr):
    max_dist = jpsp.jump_dist(c, r, dc, dr)
    if max_dist < 2:
        return None
    for step in range(1, max_dist):
        nc, nr = c + dc * step, r + dr * step
        if nc == gc and nr == gr:
            return (nc, nr)
        if _has_forced_jpsplus(jpsp, nc, nr, dc, dr):
            return (nc, nr)
    return None


def _jump_diag_jpsplus(jpsp, c, r, dc, dr, gc, gr):
    nc, nr = c + dc, r + dr
    if not jpsp.in_bounds(nc, nr) or jpsp.is_blocked(nc, nr):
        return None
    if nc == gc and nr == gr:
        return (nc, nr)

    if jpsp.is_blocked(nc - dc, nr) and not jpsp.is_blocked(nc - dc, nr + dr):
        return (nc, nr)
    if jpsp.is_blocked(nc, nr - dr) and not jpsp.is_blocked(nc + dc, nr - dr):
        return (nc, nr)

    if (_jump_cardinal_jpsplus(jpsp, nc, nr, dc, 0, gc, gr) is not None or
            _jump_cardinal_jpsplus(jpsp, nc, nr, 0, dr, gc, gr) is not None):
        return (nc, nr)

    return _jump_diag_jpsplus(jpsp, nc, nr, dc, dr, gc, gr)


def _has_forced_jpsplus(jpsp, c, r, dc, dr):
    if dr == 0:
        above = jpsp.is_blocked(c, r + 1) and not jpsp.is_blocked(c + dc, r + 1)
        below = jpsp.is_blocked(c, r - 1) and not jpsp.is_blocked(c + dc, r - 1)
        return above or below
    else:
        left = jpsp.is_blocked(c - 1, r) and not jpsp.is_blocked(c - 1, r + dr)
        right = jpsp.is_blocked(c + 1, r) and not jpsp.is_blocked(c + 1, r + dr)
        return left or right


def jps_plus_search(jpsp, start_xy, goal_xy):
    if jpsp._dirty:
        jpsp.precompute()

    sc, sr = jpsp.world_to_grid(*start_xy)
    gc, gr = jpsp.world_to_grid(*goal_xy)

    if jpsp.is_blocked(sc, sr) or jpsp.is_blocked(gc, gr):
        return []

    came_from = {}
    g_score = {(sc, sr): 0.0}
    open_heap = [(0.0, sc, sr)]
    open_set = {(sc, sr)}

    while open_heap:
        _, c, r = heapq.heappop(open_heap)
        open_set.discard((c, r))

        if (c, r) == (gc, gr):
            path = []
            key = (c, r)
            while key in came_from:
                wx, wy = jpsp.grid_to_world(*key)
                path.append((wx, wy))
                key = came_from[key]
            wx, wy = jpsp.grid_to_world(*key)
            path.append((wx, wy))
            path.reverse()
            return path

        parent = came_from.get((c, r))
        successors = _identify_successors_jpsplus(jpsp, c, r, parent, gc, gr)

        for nc, nr in successors:
            dc = nc - c
            dr = nr - r
            step_cost = (dc * dc + dr * dr) ** 0.5
            tentative_g = g_score[(c, r)] + step_cost
            key = (nc, nr)
            if key not in g_score or tentative_g < g_score[key]:
                came_from[key] = (c, r)
                g_score[key] = tentative_g
                h = ((gc - nc) ** 2 + (gr - nr) ** 2) ** 0.5
                f = tentative_g + h
                if key not in open_set:
                    heapq.heappush(open_heap, (f, nc, nr))
                    open_set.add(key)

    return []


def _identify_successors_jpsplus(jpsp, c, r, parent, gc, gr):
    successors = []
    for dc, dr in _get_directions_jpsplus(c, r, parent, jpsp):
        jp = _jump_jpsplus(jpsp, c, r, dc, dr, gc, gr)
        if jp is not None:
            successors.append(jp)
    return successors


def _get_directions_jpsplus(c, r, parent, jpsp):
    if parent is None:
        return DIRS[:]

    pc, pr = parent
    dc = _sign(c - pc)
    dr = _sign(r - pr)

    dirs = []
    if dc != 0 and dr != 0:
        dirs.append((dc, dr))
        dirs.append((dc, 0))
        dirs.append((0, dr))
        if jpsp.is_blocked(c - dc, r):
            dirs.append((-dc, 0))
            dirs.append((-dc, dr))
        if jpsp.is_blocked(c, r - dr):
            dirs.append((0, -dr))
            dirs.append((dc, -dr))
    elif dc != 0:
        dirs.append((dc, 0))
        dirs.append((dc, 1))
        dirs.append((dc, -1))
        if jpsp.is_blocked(c, r + 1):
            dirs.append((0, 1))
            dirs.append((dc, 1))
        if jpsp.is_blocked(c, r - 1):
            dirs.append((0, -1))
            dirs.append((dc, -1))
    else:
        dirs.append((0, dr))
        dirs.append((1, dr))
        dirs.append((-1, dr))
        if jpsp.is_blocked(c + 1, r):
            dirs.append((1, 0))
            dirs.append((1, dr))
        if jpsp.is_blocked(c - 1, r):
            dirs.append((-1, 0))
            dirs.append((-1, dr))

    return _unique_dirs(dirs)
