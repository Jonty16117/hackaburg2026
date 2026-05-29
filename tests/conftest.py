import pytest
import math


POOL_W = 1000
POOL_H = 200
CELL_SZ = 2
DUCK_R = 15


@pytest.fixture
def pool_dims():
    return (POOL_W, POOL_H)


@pytest.fixture
def cell_size():
    return CELL_SZ


@pytest.fixture
def duck_radius():
    return DUCK_R


@pytest.fixture
def empty_obstacles():
    return []


@pytest.fixture
def single_obstacle():
    return [{"id": 0, "x": 700.0, "y": 100.0, "r": 20.0}]


@pytest.fixture
def cluster_obstacles():
    """The 8081 scenario cluster at x≈833–862."""
    return [
        {"id": 4, "x": 840.0, "y": 115.0, "r": 7},
        {"id": 5, "x": 840.0, "y": 170.0, "r": 13},
        {"id": 6, "x": 850.0, "y": 190.0, "r": 15},
        {"id": 7, "x": 850.0, "y": 136.0, "r": 13},
    ]


@pytest.fixture
def wall_obstacles():
    """A wall spanning the corridor at y=80 to y=120."""
    return [
        {"id": 0, "x": 600.0, "y": 80.0, "r": 5},
        {"id": 1, "x": 620.0, "y": 80.0, "r": 5},
        {"id": 2, "x": 640.0, "y": 80.0, "r": 5},
        {"id": 3, "x": 660.0, "y": 80.0, "r": 5},
        {"id": 4, "x": 680.0, "y": 80.0, "r": 5},
        {"id": 5, "x": 700.0, "y": 80.0, "r": 5},
        {"id": 6, "x": 720.0, "y": 80.0, "r": 5},
        {"id": 7, "x": 740.0, "y": 80.0, "r": 5},
        {"id": 8, "x": 760.0, "y": 80.0, "r": 5},
        {"id": 9, "x": 780.0, "y": 80.0, "r": 5},
        {"id": 10, "x": 800.0, "y": 80.0, "r": 5},
        {"id": 11, "x": 820.0, "y": 120.0, "r": 5},
        {"id": 12, "x": 840.0, "y": 120.0, "r": 5},
        {"id": 13, "x": 860.0, "y": 120.0, "r": 5},
        {"id": 14, "x": 880.0, "y": 120.0, "r": 5},
        {"id": 15, "x": 900.0, "y": 120.0, "r": 5},
    ]


@pytest.fixture
def grid_kwargs():
    return {"width_cm": POOL_W, "height_cm": POOL_H, "cell_size": CELL_SZ, "duck_radius": DUCK_R}
