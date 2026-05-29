import math
import pytest
from navigation.grid import Grid


class TestGridDimensions:
    def test_dimensions_2cm_cells(self, pool_dims, cell_size):
        g = Grid(*pool_dims, cell_size, [], 15)
        assert g.w == 500, f"Expected 500 cols, got {g.w}"
        assert g.h == 100, f"Expected 100 rows, got {g.h}"

    def test_coverage(self, pool_dims, cell_size):
        g = Grid(*pool_dims, cell_size, [], 15)
        assert g.grid_to_world(0, 0)[0] > 0  # first cell center positive
        assert g.grid_to_world(g.w - 1, g.h - 1)[0] < pool_dims[0]
        assert g.grid_to_world(g.w - 1, g.h - 1)[1] < pool_dims[1]


class TestWorldToGrid:
    @pytest.mark.parametrize("wx, wy, ec, er", [
        (0, 0, 0, 0),
        (2, 0, 1, 0),
        (500, 100, 250, 50),
        (999, 199, 499, 99),
        (1000, 200, 500, 100),
    ])
    def test_world_to_grid(self, pool_dims, cell_size, wx, wy, ec, er):
        g = Grid(*pool_dims, cell_size, [], 15)
        c, r = g.world_to_grid(wx, wy)
        assert c == ec, f"Expected col={ec}, got {c} for x={wx}"
        assert r == er, f"Expected row={er}, got {r} for y={wy}"


class TestGridToWorld:
    def test_grid_to_world_center(self, pool_dims, cell_size):
        g = Grid(*pool_dims, cell_size, [], 15)
        x, y = g.grid_to_world(0, 0)
        assert x == 1.0, f"Expected x=1.0, got {x}"
        assert y == 1.0, f"Expected y=1.0, got {y}"

    def test_grid_to_world_mid(self, pool_dims, cell_size):
        g = Grid(*pool_dims, cell_size, [], 15)
        x, y = g.grid_to_world(250, 50)
        assert x == 501.0, f"Expected x=501.0, got {x}"
        assert y == 101.0, f"Expected y=101.0, got {y}"

    def test_grid_to_world_corner(self, pool_dims, cell_size):
        g = Grid(*pool_dims, cell_size, [], 15)
        x, y = g.grid_to_world(499, 99)
        assert x == 999.0, f"Expected x=999.0, got {x}"
        assert y == 199.0, f"Expected y=199.0, got {y}"

    def test_roundtrip(self, pool_dims, cell_size):
        g = Grid(*pool_dims, cell_size, [], 15)
        for wx, wy in [(100, 50), (500, 100), (900, 150)]:
            c, r = g.world_to_grid(wx, wy)
            rx, ry = g.grid_to_world(c, r)
            # Cell center should be within cell_size/2 of original
            assert abs(rx - wx) <= cell_size / 2
            assert abs(ry - wy) <= cell_size / 2


class TestEmptyGrid:
    def test_all_cells_free(self, pool_dims, cell_size):
        g = Grid(*pool_dims, cell_size, [], 15)
        for r in range(g.h):
            for c in range(g.w):
                assert not g.is_blocked(c, r), f"Cell ({c},{r}) should be free"


class TestObstacleBlocking:
    def test_obstacle_blocks_nearby_cells(self):
        g = Grid(1000, 200, 10, [{"x": 500, "y": 100, "r": 20}], 15)
        # Cells within 35 = 15+20 should be blocked
        cx, cy = g.world_to_grid(500, 100)
        # Check four cells around center
        for dc, dr in [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)]:
            nc, nr = cx + dc, cy + dr
            if g.in_bounds(nc, nr):
                assert g.is_blocked(nc, nr), f"Cell ({nc},{nr}) near obstacle should be blocked"
        # Far away cell should be free
        assert not g.is_blocked(0, 0), "Far cell (0,0) should be free"

    def test_inflation_by_duck_radius(self, pool_dims, cell_size):
        """Duck radius 15 + obstacle r=10 = 25cm blocking radius."""
        g = Grid(*pool_dims, cell_size, [{"x": 500, "y": 100, "r": 10}], 15)
        cx, cy = g.world_to_grid(500, 100)
        # At cell_size=2, cell center at (501, 101)
        # Distance from obstacle center (500, 100) to (501, 101) = sqrt(2) ≈ 1.41
        # 1.41 < 25, so should be blocked
        assert g.is_blocked(cx, cy), "Cell containing obstacle center should be blocked"
        # Cell far away should be free
        assert not g.is_blocked(0, 0), "Far cell should be free"


class TestInBounds:
    def test_in_bounds_inside(self):
        g = Grid(1000, 200, 2, [], 15)
        assert g.in_bounds(0, 0) is True
        assert g.in_bounds(499, 99) is True
        assert g.in_bounds(250, 50) is True

    def test_out_of_bounds(self):
        g = Grid(1000, 200, 2, [], 15)
        assert g.in_bounds(-1, 0) is False
        assert g.in_bounds(0, -1) is False
        assert g.in_bounds(500, 50) is False
        assert g.in_bounds(250, 100) is False

    def test_out_of_bounds_is_blocked(self):
        g = Grid(1000, 200, 2, [], 15)
        assert g.is_blocked(-1, 0) is True
        assert g.is_blocked(500, 50) is True
        assert g.is_blocked(0, 100) is True


class TestNeighbors:
    def test_neighbors_count_8(self):
        g = Grid(1000, 200, 2, [], 15)
        neighbors = list(g.neighbors(250, 50))
        assert len(neighbors) == 8

    def test_neighbors_in_corner(self):
        g = Grid(1000, 200, 2, [], 15)
        neighbors = list(g.neighbors(0, 0))
        assert len(neighbors) == 3  # (1,0), (0,1), (1,1)
        assert (1, 0) in neighbors
        assert (0, 1) in neighbors
        assert (1, 1) in neighbors


class TestBuildPerformance:
    def test_build_doesnt_error(self, grid_kwargs, cluster_obstacles):
        g = Grid(**grid_kwargs, obstacles=cluster_obstacles)
        blocked_count = sum(
            1 for r in range(g.h) for c in range(g.w) if g.is_blocked(c, r)
        )
        assert blocked_count > 0
        assert blocked_count < g.w * g.h  # not everything blocked
