import math
import pytest
from navigation.utils import clamp, normalize_angle, heading_error


@pytest.mark.parametrize(
    "v, lo, hi, expected",
    [
        (5, 0, 10, 5),
        (-5, 0, 10, 0),
        (15, 0, 10, 10),
        (0, 0, 10, 0),
        (10, 0, 10, 10),
        (-100, -50, 50, -50),
    ],
)
def test_clamp(v, lo, hi, expected):
    assert clamp(v, lo, hi) == expected


@pytest.mark.parametrize(
    "angle, expected",
    [
        (0, 0),
        (math.pi, math.pi),
        (-math.pi, -math.pi),
        (3 * math.pi, math.pi),
        (-3 * math.pi, -math.pi),
        (math.pi / 2, math.pi / 2),
        (-math.pi / 2, -math.pi / 2),
    ],
)
def test_normalize_angle(angle, expected):
    assert abs(normalize_angle(angle) - expected) < 1e-6


@pytest.mark.parametrize(
    "target, current, abs_expected",
    [
        (0.5, 0.3, 0.2),
        (0.1, 0.1, 0.0),
        (0.1, 6.2, 0.183),
        (0, math.pi, math.pi),
        (math.pi, 0, math.pi),
    ],
)
def test_heading_error_abs(target, current, abs_expected):
    err = heading_error(target, current)
    assert abs(abs(err) - abs_expected) < 0.01


def test_heading_error_symmetry():
    a, b = 0.3, 1.7
    err_ab = heading_error(a, b)
    err_ba = heading_error(b, a)
    assert abs(err_ab + err_ba) < 1e-6


def test_heading_error_range():
    for _ in range(100):
        import random
        random.seed(42)
        t = random.uniform(-math.pi * 2, math.pi * 2)
        c = random.uniform(-math.pi * 2, math.pi * 2)
        err = heading_error(t, c)
        assert -math.pi <= err <= math.pi
