import math
import pytest
from navigation.grid import Grid
from navigation.jps import jps_search, jps_plus_search, JPSPlusGrid

POOL_W = 1000
POOL_H = 200
CELL_SZ = 2
DUCK_R = 15


def _straight_line_path(g, start, goal):
    """Check if path is a straight line (all waypoints lie on the line)."""
    if len(g) <= 1:
        return True
    sx, sy = g[0]
    gx, gy = g[-1]
    dx, dy = gx - sx, gy - sy
    for wx, wy in g:
        cross = abs(dx * (wy - sy) - dy * (wx - sx))
        if cross > CELL_SZ:
            return False
    return True


class TestJPSNoObstacles:
    def test_straight_line_path_found(self):
        g = Grid(POOL_W, POOL_H, CELL_SZ, [], DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        assert len(path) > 0, "Should find a path"
        assert _straight_line_path(path, (500, 100), (900, 100)), "Path should be straight"

    def test_last_waypoint_near_goal(self):
        g = Grid(POOL_W, POOL_H, CELL_SZ, [], DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        gx, gy = path[-1]
        assert math.hypot(gx - 900, gy - 100) < CELL_SZ * 2, "Last waypoint should be near goal"


class TestJPSSingleObstacle:
    def test_path_diverges_around_obstacle(self):
        g = Grid(POOL_W, POOL_H, CELL_SZ, [{"x": 700, "y": 100, "r": 30}], DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        assert len(path) > 0, "Should find a path around obstacle"
        # Path should not be straight line (must go around)
        straight = _straight_line_path(path, (500, 100), (900, 100))
        assert not straight, "Path should not be a straight line (obstacle in the way)"

    def test_path_does_not_intersect_obstacle(self):
        g = Grid(POOL_W, POOL_H, CELL_SZ, [{"x": 700, "y": 100, "r": 30}], DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        for wx, wy in path:
            d = math.hypot(wx - 700, wy - 100)
            assert d > DUCK_R + 30, f"Waypoint ({wx:.1f},{wy:.1f}) too close to obstacle"

    def test_path_reaches_goal(self):
        g = Grid(POOL_W, POOL_H, CELL_SZ, [{"x": 700, "y": 100, "r": 30}], DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        gx, gy = path[-1]
        assert math.hypot(gx - 900, gy - 100) < CELL_SZ * 2


class TestJPSClusterObstacle:
    def test_path_found_through_cluster(self):
        obstacles = [
            {"x": 840, "y": 115, "r": 7},
            {"x": 840, "y": 170, "r": 13},
            {"x": 850, "y": 190, "r": 15},
            {"x": 850, "y": 136, "r": 13},
        ]
        g = Grid(POOL_W, POOL_H, CELL_SZ, obstacles, DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        assert len(path) > 0, "Should find path around cluster"
        gx, gy = path[-1]
        assert math.hypot(gx - 900, gy - 100) < CELL_SZ * 4

    def test_path_below_or_above_cluster(self):
        """Path through cluster area must not pass through blocked cells."""
        obstacles = [
            {"x": 840, "y": 115, "r": 7},
            {"x": 840, "y": 170, "r": 13},
            {"x": 850, "y": 190, "r": 15},
            {"x": 850, "y": 136, "r": 13},
        ]
        g = Grid(POOL_W, POOL_H, CELL_SZ, obstacles, DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        assert len(path) > 0
        for wx, wy in path:
            c, r = g.world_to_grid(wx, wy)
            assert not g.is_blocked(c, r), (
                f"Waypoint ({wx:.1f},{wy:.1f}) is in blocked cell ({c},{r})"
            )


class TestJPSUnreachable:
    def test_wall_returns_empty(self):
        obstacles = []
        for x in range(100, 901, 50):
            obstacles.append({"x": float(x), "y": 100, "r": 20})
        g = Grid(POOL_W, POOL_H, CELL_SZ, obstacles, DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        assert len(path) == 0, "Should return empty path when blocked"


class TestJPSStartBlocked:
    def test_start_on_obstacle_returns_empty(self):
        g = Grid(POOL_W, POOL_H, CELL_SZ, [{"x": 500, "y": 100, "r": 30}], DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        assert len(path) == 0, "Should return empty when start is blocked"


class TestJPSSameStartGoal:
    def test_same_point(self):
        g = Grid(POOL_W, POOL_H, CELL_SZ, [], DUCK_R)
        path = jps_search(g, (500, 100), (500, 100))
        assert len(path) >= 1, "Should have at least start point"
        gx, gy = path[-1]
        assert math.hypot(gx - 500, gy - 100) < CELL_SZ

    def test_same_point_blocked(self):
        g = Grid(POOL_W, POOL_H, CELL_SZ, [{"x": 500, "y": 100, "r": 30}], DUCK_R)
        path = jps_search(g, (500, 100), (500, 100))
        assert len(path) == 0, "Should return empty when start=goal and cell is blocked"


class TestJPSPathQuality:
    def test_path_waypoints_not_inside_obstacles(self):
        obstacles = [{"x": 700, "y": 100, "r": 30}]
        g = Grid(POOL_W, POOL_H, CELL_SZ, obstacles, DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        for wx, wy in path:
            d = math.hypot(wx - 700, wy - 100)
            assert d > DUCK_R + 30 - 1, f"Waypoint ({wx:.1f},{wy:.1f}) inside obstacle"

    def test_path_starts_near_start(self):
        g = Grid(POOL_W, POOL_H, CELL_SZ, [], DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        sx, sy = path[0]
        assert math.hypot(sx - 500, sy - 100) < CELL_SZ * 2


class TestJPSPlus:
    def test_precompute_no_error(self):
        g = JPSPlusGrid(POOL_W, POOL_H, CELL_SZ, [], DUCK_R)
        g.precompute()
        assert hasattr(g, "_jump_dist")

    def test_precompute_with_obstacles(self):
        g = JPSPlusGrid(POOL_W, POOL_H, CELL_SZ,
                       [{"x": 700, "y": 100, "r": 20}], DUCK_R)
        g.precompute()

    def test_jps_plus_matches_jps(self):
        obstacles = [{"x": 700, "y": 100, "r": 30}]
        g_jps = Grid(POOL_W, POOL_H, CELL_SZ, obstacles, DUCK_R)
        g_jpsp = JPSPlusGrid(POOL_W, POOL_H, CELL_SZ, obstacles, DUCK_R)
        g_jpsp.precompute()

        path_jps = jps_search(g_jps, (500, 100), (900, 100))
        path_jpsp = jps_plus_search(g_jpsp, (500, 100), (900, 100))

        assert len(path_jpsp) == len(path_jps), (
            f"Path lengths differ: JPS={len(path_jps)}, JPS+={len(path_jpsp)}"
        )
        for (x1, y1), (x2, y2) in zip(path_jps, path_jpsp):
            assert abs(x1 - x2) < 0.1 and abs(y1 - y2) < 0.1, (
                f"Waypoints differ: JPS=({x1:.1f},{y1:.1f}) JPS+=({x2:.1f},{y2:.1f})"
            )

    def test_jps_plus_empty_matches_jps(self):
        g_jps = Grid(POOL_W, POOL_H, CELL_SZ, [], DUCK_R)
        g_jpsp = JPSPlusGrid(POOL_W, POOL_H, CELL_SZ, [], DUCK_R)
        g_jpsp.precompute()

        path_jps = jps_search(g_jps, (500, 100), (900, 100))
        path_jpsp = jps_plus_search(g_jpsp, (500, 100), (900, 100))

        assert len(path_jpsp) == len(path_jps)
        for (x1, y1), (x2, y2) in zip(path_jps, path_jpsp):
            assert abs(x1 - x2) < 0.1 and abs(y1 - y2) < 0.1

    def test_jps_plus_recompute(self):
        obstacles = [{"x": 700, "y": 100, "r": 20}]
        g = JPSPlusGrid(POOL_W, POOL_H, CELL_SZ, obstacles, DUCK_R)
        g.precompute()
        path1 = jps_plus_search(g, (500, 100), (900, 100))
        assert len(path1) > 0

        g2 = JPSPlusGrid(POOL_W, POOL_H, CELL_SZ, [], DUCK_R)
        g2.precompute()
        path2 = jps_plus_search(g2, (500, 100), (900, 100))
        assert len(path2) > 0
        assert _straight_line_path(path2, (500, 100), (900, 100)), (
            "Without obstacles, path should be straight"
        )

    def test_jps_plus_unreachable(self):
        obstacles = []
        for x in range(100, 901, 50):
            obstacles.append({"x": float(x), "y": 100, "r": 20})
        g = JPSPlusGrid(POOL_W, POOL_H, CELL_SZ, obstacles, DUCK_R)
        g.precompute()
        path = jps_plus_search(g, (500, 100), (900, 100))
        assert len(path) == 0


class TestJPSPlusGridClass:
    def test_inherits_grid(self):
        g = JPSPlusGrid(1000, 200, 2, [], 15)
        assert g.w == 500
        assert g.h == 100

    def test_precompute_twice(self):
        g = JPSPlusGrid(1000, 200, 2, [], 15)
        g.precompute()
        g.precompute()  # should not error
