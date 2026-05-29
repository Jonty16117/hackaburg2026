import math
from navigation.odometry import Odometry
from navigation.config import START_X_CM, START_Y_CM


def test_odometry_init():
    o = Odometry(START_X_CM, START_Y_CM, 0, 30, 100)
    x, y, theta = o.position()
    assert x == START_X_CM
    assert y == START_Y_CM
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
    o = Odometry(START_X_CM, START_Y_CM, 0.5, 30, 100)
    o.update(0, 0, 1.0)
    x, y, theta = o.position()
    assert abs(x - START_X_CM) < 0.1
    assert abs(y - START_Y_CM) < 0.1
    assert abs(theta - 0.5) < 0.1


def test_odometry_theta_wraps():
    o = Odometry(0, 0, 3.0, 30, 100)
    o.update(0.5, -0.5, 1.0)
    x, y, theta = o.position()
    assert -math.pi <= theta <= math.pi
