import math
from navigation.grid import Grid
from navigation.jps import jps_search

POOL_W = 1000
POOL_H = 200
CELL_SZ = 2
DUCK_R = 15


def _straight_line_path(path, sx, sy):
    if len(path) <= 1:
        return True
    gx, gy = path[-1]
    dx, dy = gx - sx, gy - sy
    for wx, wy in path:
        cross = abs(dx * (wy - sy) - dy * (wx - sx))
        if cross > CELL_SZ:
            return False
    return True


class TestJPSNoObstacles:
    def test_straight_line_path_found(self):
        g = Grid(POOL_W, POOL_H, CELL_SZ, [], DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        assert len(path) > 0
        assert _straight_line_path(path, path[0][0], path[0][1])

    def test_last_waypoint_near_goal(self):
        g = Grid(POOL_W, POOL_H, CELL_SZ, [], DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        gx, gy = path[-1]
        assert math.hypot(gx - 900, gy - 100) < CELL_SZ * 2


class TestJPSSingleObstacle:
    def test_path_avoids_obstacle(self):
        g = Grid(POOL_W, POOL_H, CELL_SZ, [{"x": 700, "y": 100, "r": 30}], DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        assert len(path) > 0
        assert not _straight_line_path(path, path[0][0], path[0][1])

    def test_path_clear_of_obstacle(self):
        g = Grid(POOL_W, POOL_H, CELL_SZ, [{"x": 700, "y": 100, "r": 30}], DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        for wx, wy in path:
            assert math.hypot(wx - 700, wy - 100) > DUCK_R + 30

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
        assert len(path) > 0
        gx, gy = path[-1]
        assert math.hypot(gx - 900, gy - 100) < CELL_SZ * 4

    def test_path_does_not_enter_blocked_cells(self):
        obstacles = [
            {"x": 840, "y": 115, "r": 7},
            {"x": 840, "y": 170, "r": 13},
            {"x": 850, "y": 190, "r": 15},
            {"x": 850, "y": 136, "r": 13},
        ]
        g = Grid(POOL_W, POOL_H, CELL_SZ, obstacles, DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        for wx, wy in path:
            c, r = g.world_to_grid(wx, wy)
            assert not g.is_blocked(c, r)


class TestJPSUnreachable:
    def test_wall_returns_empty(self):
        obstacles = [{"x": float(x), "y": 100, "r": 20} for x in range(100, 901, 50)]
        g = Grid(POOL_W, POOL_H, CELL_SZ, obstacles, DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        assert len(path) == 0


class TestJPSStartBlocked:
    def test_start_on_obstacle_returns_empty(self):
        g = Grid(POOL_W, POOL_H, CELL_SZ, [{"x": 500, "y": 100, "r": 30}], DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        assert len(path) == 0


class TestJPSSameStartGoal:
    def test_same_point(self):
        g = Grid(POOL_W, POOL_H, CELL_SZ, [], DUCK_R)
        path = jps_search(g, (500, 100), (500, 100))
        assert len(path) >= 1
        gx, gy = path[-1]
        assert math.hypot(gx - 500, gy - 100) < CELL_SZ

    def test_same_point_blocked(self):
        g = Grid(POOL_W, POOL_H, CELL_SZ, [{"x": 500, "y": 100, "r": 30}], DUCK_R)
        path = jps_search(g, (500, 100), (500, 100))
        assert len(path) == 0


class TestJPSPathQuality:
    def test_waypoints_start_near_origin(self):
        g = Grid(POOL_W, POOL_H, CELL_SZ, [], DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        sx, sy = path[0]
        assert math.hypot(sx - 500, sy - 100) < CELL_SZ * 2

    def test_waypoints_not_in_obstacle(self):
        obstacles = [{"x": 700, "y": 100, "r": 30}]
        g = Grid(POOL_W, POOL_H, CELL_SZ, obstacles, DUCK_R)
        path = jps_search(g, (500, 100), (900, 100))
        for wx, wy in path:
            assert math.hypot(wx - 700, wy - 100) > DUCK_R + 30 - 1
