import math
from navigation.perimeter import Perimeter
from navigation.config import PERIMETER_CM


def test_is_inside_center():
    p = Perimeter(PERIMETER_CM)
    assert p.is_inside(500, 100)


def test_is_inside_corner():
    p = Perimeter(PERIMETER_CM)
    assert p.is_inside(1, 1)


def test_is_outside():
    p = Perimeter(PERIMETER_CM)
    assert not p.is_inside(-10, 100)
    assert not p.is_inside(500, -10)
    assert not p.is_inside(1010, 100)
    assert not p.is_inside(500, 210)


def test_is_outside_exact_boundary():
    p = Perimeter(PERIMETER_CM)
    assert not p.is_inside(-1, 100)


def test_distance_to_edge_center():
    p = Perimeter(PERIMETER_CM)
    d = p.distance_to_edge(500, 100)
    assert abs(d - 100) < 0.01


def test_distance_to_edge_near():
    p = Perimeter(PERIMETER_CM)
    d = p.distance_to_edge(500, 10)
    assert abs(d - 10) < 0.01


def test_is_near_edge():
    p = Perimeter(PERIMETER_CM)
    assert p.is_near_edge(500, 10, margin=30)
    assert not p.is_near_edge(500, 100, margin=30)


def test_bearing_to_center():
    p = Perimeter(PERIMETER_CM)
    bearing = p.bearing_to_center(200, 50)
    assert bearing > 0
    assert bearing < math.pi / 2


def test_bearing_to_center_from_below():
    p = Perimeter(PERIMETER_CM)
    bearing = p.bearing_to_center(500, 50)
    assert 0 < bearing < math.pi


def test_bearing_to_center_from_center():
    p = Perimeter(PERIMETER_CM)
    bearing = p.bearing_to_center(500, 100)
    assert abs(bearing) < 1e-6


def test_centroid():
    p = Perimeter(PERIMETER_CM)
    cx, cy = p.centroid
    assert abs(cx - 500) < 0.01
    assert abs(cy - 100) < 0.01


def test_bounds():
    p = Perimeter(PERIMETER_CM)
    assert p.min_x == 0
    assert p.max_x == 1000
    assert p.min_y == 0
    assert p.max_y == 200
