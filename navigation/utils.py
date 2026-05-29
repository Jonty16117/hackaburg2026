"""Shared math utilities used across navigation and simulation."""

import math


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def heading_error(target, current):
    e = target - current
    return ((e + math.pi) % (2 * math.pi)) - math.pi
