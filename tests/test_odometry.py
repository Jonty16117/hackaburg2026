import math
from navigation.odometry import Odometry


def test_odometry_init():
    o = Odometry(500, 100, 0, 30, 100)
    x, y, theta = o.position()
    assert x == 500
    assert y == 100
    assert theta == 0


def test_odometry_straight():
    o = Odometry(0, 0, 0, 30, 100)
    o.update(0.5, 0.5, 1.0)
    x, y, theta = o.position()
    assert abs(x - 50) < 0.1
    assert abs(y) < 0.1
    assert abs(theta) < 0.1


def test_odometry_turn_in_place():
    o = Odometry(0, 0, 0, 30, 100)
    o.update(-0.3, 0.3, 0.5)
    x, y, theta = o.position()
    assert abs(x) < 0.1
    assert abs(y) < 0.1
    assert abs(theta - 1.0) < 0.1


def test_odometry_forward_with_turn():
    o = Odometry(0, 0, 0, 30, 100)
    o.update(0.3, 0.5, 0.5)
    x, y, theta = o.position()
    assert x > 0
    assert theta > 0


def test_odometry_zero_speeds():
    o = Odometry(500, 100, 0.5, 30, 100)
    o.update(0, 0, 1.0)
    x, y, theta = o.position()
    assert abs(x - 500) < 0.1
    assert abs(y - 100) < 0.1
    assert abs(theta - 0.5) < 0.1


def test_odometry_theta_wraps():
    o = Odometry(0, 0, 3.0, 30, 100)
    o.update(0.5, -0.5, 1.0)
    x, y, theta = o.position()
    assert -math.pi <= theta <= math.pi
