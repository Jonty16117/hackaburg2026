"""Jump Point Search for grid path planning."""

import heapq


DIRS = [(1, 0), (1, 1), (0, 1), (-1, 1),
        (-1, 0), (-1, -1), (0, -1), (1, -1)]


def _sign(x):
    if x > 0:
        return 1
    if x < 0:
        return -1
    return 0


def _unique_dirs(dirs):
    seen = set()
    result = []
    for d in dirs:
        if d not in seen:
            seen.add(d)
            result.append(d)
    return result


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
            dc, dr = nc - c, nr - r
            tentative_g = g_score[(c, r)] + (dc * dc + dr * dr) ** 0.5
            key = (nc, nr)
            if key not in g_score or tentative_g < g_score[key]:
                came_from[key] = (c, r)
                g_score[key] = tentative_g
                h = ((gc - nc) ** 2 + (gr - nr) ** 2) ** 0.5
                if key not in open_set:
                    heapq.heappush(open_heap, (tentative_g + h, nc, nr))
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
    dc, dr = _sign(c - pc), _sign(r - pr)
    dirs = []

    if dc != 0 and dr != 0:
        dirs.extend([(dc, dr), (dc, 0), (0, dr)])
        if grid.is_blocked(c - dc, r):
            dirs.extend([(-dc, 0), (-dc, dr)])
        if grid.is_blocked(c, r - dr):
            dirs.extend([(0, -dr), (dc, -dr)])
    elif dc != 0:
        dirs.extend([(dc, 0), (dc, 1), (dc, -1)])
        if grid.is_blocked(c, r + 1):
            dirs.extend([(0, 1), (dc, 1)])
        if grid.is_blocked(c, r - 1):
            dirs.extend([(0, -1), (dc, -1)])
    else:
        dirs.extend([(0, dr), (1, dr), (-1, dr)])
        if grid.is_blocked(c + 1, r):
            dirs.extend([(1, 0), (1, dr)])
        if grid.is_blocked(c - 1, r):
            dirs.extend([(-1, 0), (-1, dr)])

    return _unique_dirs(dirs)


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
        if _jump(grid, nc, nr, dc, 0, gc, gr) is not None or \
                _jump(grid, nc, nr, 0, dr, gc, gr) is not None:
            return (nc, nr)
    else:
        if _has_forced(grid, nc, nr, dc, dr):
            return (nc, nr)

    return _jump(grid, nc, nr, dc, dr, gc, gr)


def _has_forced(grid, c, r, dc, dr):
    if dr == 0:
        return ((grid.is_blocked(c, r + 1) and not grid.is_blocked(c + dc, r + 1)) or
                (grid.is_blocked(c, r - 1) and not grid.is_blocked(c + dc, r - 1)))
    else:
        return ((grid.is_blocked(c - 1, r) and not grid.is_blocked(c - 1, r + dr)) or
                (grid.is_blocked(c + 1, r) and not grid.is_blocked(c + 1, r + dr)))


def _reconstruct_path(came_from, c, r, grid):
    path = []
    key = (c, r)
    while key in came_from:
        path.append(grid.grid_to_world(*key))
        key = came_from[key]
    path.append(grid.grid_to_world(*key))
    path.reverse()
    return path
