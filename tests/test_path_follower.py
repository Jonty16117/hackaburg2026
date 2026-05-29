import math
import pytest
from navigation.path_follower import PathFollower


class TestStraightPath:
    def test_follow_east(self):
        path = [(500, 100), (600, 100), (700, 100), (800, 100), (900, 100)]
        pf = PathFollower(path, max_speed=0.6, heading_tolerance=0.02)
        ls, rs = pf.compute_speeds(500, 100, 0.0)  # facing east
        assert ls > 0 and rs > 0, "Should go forward"
        assert abs(ls - rs) < 0.01, "Should go straight"

    def test_advance_to_next_waypoint(self):
        path = [(500, 100), (550, 100), (900, 100)]
        pf = PathFollower(path, max_speed=0.6, heading_tolerance=0.02)
        pf.compute_speeds(500, 100, 0.0)
        assert pf.waypoint_index == 1
        pf.compute_speeds(510, 100, 0.0)
        assert pf.waypoint_index == 1
        pf.compute_speeds(535, 100, 0.0)
        assert pf.waypoint_index == 2

    def test_waypoint_not_skipped(self):
        path = [(500, 100), (550, 100), (900, 100)]
        pf = PathFollower(path, max_speed=0.6, heading_tolerance=0.02)
        pf.compute_speeds(500, 100, 0.0)
        pf.compute_speeds(535, 100, 0.0)
        assert pf.waypoint_index == 2

    def test_advance_through_path(self):
        path = [(500, 100), (550, 100), (600, 100), (900, 100)]
        pf = PathFollower(path, max_speed=0.6, heading_tolerance=0.02)
        pf.compute_speeds(500, 100, 0.0)
        pf.compute_speeds(535, 100, 0.0)
        assert pf.waypoint_index == 2
        pf.compute_speeds(585, 100, 0.0)
        assert pf.waypoint_index == 3


class TestTurning:
    def test_turn_left(self):
        """Waypoint is north-east, duck facing east → need to turn left (counter-clockwise)."""
        path = [(500, 100), (600, 150)]
        pf = PathFollower(path, max_speed=0.6, heading_tolerance=0.02)
        ls, rs = pf.compute_speeds(500, 100, 0.0)  # facing east, goal is NE
        # Should turn left: left speed < right speed
        assert ls < rs, "Should turn left (counter-clockwise)"

    def test_turn_right(self):
        """Waypoint is south-east, duck facing east → need to turn right."""
        path = [(500, 100), (600, 50)]
        pf = PathFollower(path, max_speed=0.6, heading_tolerance=0.02)
        ls, rs = pf.compute_speeds(500, 100, 0.0)  # facing east, goal is SE
        assert ls > rs, "Should turn right (clockwise)"

    def test_turn_180(self):
        """Waypoint is behind the duck (west, same y), duck facing east."""
        path = [(500, 100), (400, 100)]
        pf = PathFollower(path, max_speed=0.6, heading_tolerance=0.02)
        ls, rs = pf.compute_speeds(500, 100, 0.0)  # facing east, goal is west
        # Should turn 180 — either speed can be negative depending on implementation
        # What matters is the heading error is handled correctly (turn in place)
        assert ls != rs, "Should be turning, not going straight"


class TestArrival:
    def test_arrived_at_last_waypoint(self):
        path = [(500, 100)]
        pf = PathFollower(path, max_speed=0.6, heading_tolerance=0.02)
        ls, rs = pf.compute_speeds(500, 100, 0.0)
        assert ls == 0 and rs == 0, "Should stop at arrival"
        assert pf.arrived is True

    def test_arrived_near_last_waypoint(self):
        path = [(500, 100), (550, 100)]
        pf = PathFollower(path, max_speed=0.6, heading_tolerance=0.02)
        pf.compute_speeds(500, 100, 0.0)
        pf.compute_speeds(540, 100, 0.0)  # close to (550,100)
        ls, rs = pf.compute_speeds(548, 100, 0.0)
        assert ls == 0 and rs == 0, "Should stop when close to final waypoint"
        assert pf.arrived is True

    def test_not_arrived_mid_path(self):
        path = [(500, 100), (700, 100), (900, 100)]
        pf = PathFollower(path, max_speed=0.6, heading_tolerance=0.02)
        ls, rs = pf.compute_speeds(500, 100, 0.0)
        assert pf.arrived is False


class TestEmptyPath:
    def test_empty_path_arrived(self):
        pf = PathFollower([], max_speed=0.6, heading_tolerance=0.02)
        ls, rs = pf.compute_speeds(500, 100, 0.0)
        assert ls == 0 and rs == 0
        assert pf.arrived is True


class TestNearEnough:
    def test_arrival_distance_configurable(self):
        path = [(500, 100), (900, 100)]
        pf = PathFollower(path, max_speed=0.6, heading_tolerance=0.02, arrival_dist=50)
        ls, rs = pf.compute_speeds(500, 100, 0.0)
        assert not pf.arrived
        pf.compute_speeds(860, 100, 0.0)  # within 50 of (900,100)
        assert pf.arrived
