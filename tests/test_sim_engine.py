import math
import pytest
from dashboard.sim_engine import SimEngine


def _setup_engine(**kwargs):
    e = SimEngine()
    for k, v in kwargs.items():
        setattr(e, k, v)
    return e


class TestSimEngineState:
    def test_init_state(self):
        e = SimEngine()
        s = e.get_state()
        assert s["x_cm"] == 500
        assert s["y_cm"] == 100
        assert s["autopilot"] is False
        assert s["arrived"] is False
        assert s["avoid_state"] == "none"

    def test_set_pose(self):
        e = SimEngine()
        e.set_pose(200, 150, 0.5)
        s = e.get_state()
        assert s["x_cm"] == 200
        assert s["y_cm"] == 150
        assert abs(s["theta_rad"] - 0.5) < 0.01

    def test_set_pose_resets_autopilot(self):
        e = SimEngine()
        e.set_autopilot(True)
        e.set_pose(200, 100)
        s = e.get_state()
        assert s["autopilot"] is False

    def test_reset(self):
        e = SimEngine()
        e.set_pose(200, 150)
        e.reset()
        s = e.get_state()
        assert s["x_cm"] == 500
        assert s["y_cm"] == 100

    def test_set_speeds(self):
        e = SimEngine()
        e.set_speeds(0.5, 0.3)
        s = e.get_state()
        assert s["left_speed"] == 0.5
        assert s["right_speed"] == 0.3
        assert s["autopilot"] is False

    def test_set_speeds_clamped(self):
        e = SimEngine()
        e.set_speeds(2.0, -2.0)
        s = e.get_state()
        assert s["left_speed"] == 1.0
        assert s["right_speed"] == -1.0


class TestSimEngineObstacles:
    def test_add_obstacle(self):
        e = SimEngine()
        oid = e.add_obstacle(400, 100, 25)
        obs = e.get_obstacles()
        assert len(obs) == 1
        assert obs[0]["id"] == oid
        assert obs[0]["x"] == 400
        assert obs[0]["y"] == 100
        assert obs[0]["r"] == 25

    def test_add_obstacle_default_radius(self):
        e = SimEngine()
        oid = e.add_obstacle(400, 100)
        obs = e.get_obstacles()
        assert obs[0]["r"] == 10

    def test_remove_obstacle(self):
        e = SimEngine()
        oid = e.add_obstacle(400, 100)
        assert e.remove_obstacle(oid)
        assert len(e.get_obstacles()) == 0

    def test_remove_nonexistent(self):
        e = SimEngine()
        assert not e.remove_obstacle(999)

    def test_clear_obstacles(self):
        e = SimEngine()
        e.add_obstacle(300, 50)
        e.add_obstacle(600, 150)
        e.clear_obstacles()
        assert len(e.get_obstacles()) == 0


class TestSimEngineConfig:
    def test_get_config(self):
        e = SimEngine()
        c = e.get_config()
        assert "OBST_TH" in c
        assert "MARGIN" in c
        assert "max_speed" in c

    def test_update_config(self):
        e = SimEngine()
        e.update_config({"max_speed": 0.8, "OBST_TH": 50})
        c = e.get_config()
        assert c["max_speed"] == 0.8
        assert c["OBST_TH"] == 50

    def test_set_goal(self):
        e = SimEngine()
        e.set_goal(start={"x": 100, "y": 100}, end={"x": 900, "y": 100})
        s = e.get_state()
        assert s["start"]["x"] == 100
        assert s["end"]["x"] == 900

    def test_set_goal_moves_duck(self):
        e = SimEngine()
        e.set_goal(start={"x": 200, "y": 150})
        s = e.get_state()
        assert s["x_cm"] == 200
        assert s["y_cm"] == 150


class TestSimEngineAutopilot:
    def test_autopilot_start(self):
        e = SimEngine()
        e.set_autopilot(True)
        s = e.get_state()
        assert s["autopilot"] is True

    def test_autopilot_stop(self):
        e = SimEngine()
        e.set_autopilot(True)
        e.set_autopilot(False)
        s = e.get_state()
        assert s["autopilot"] is False
        assert s["left_speed"] == 0
        assert s["right_speed"] == 0

    def test_step_advances_frame(self):
        e = SimEngine()
        e.step(0.05)
        s = e.get_state()
        assert s["frame"] == 1

    def test_sonar_override(self):
        e = SimEngine()
        e.set_autopilot(True)
        e.set_sonar_override(front=30, left=40, right=None)
        e.step(0.05)
        s = e.get_state()
        assert s["sonar_front"] == 30
        assert s["sonar_left"] == 40


class TestSimEngineNavigation:
    @pytest.mark.parametrize("desc,obstacles,start,goal,max_frames", [
        ("single r=30 center", [(400, 100, 30)], (100, 100), (900, 100), 500),
        ("single r=20 center", [(400, 100, 20)], (100, 100), (900, 100), 500),
        ("single r=10 center", [(400, 100, 10)], (100, 100), (900, 100), 500),
        ("offset r=15 above", [(400, 60, 15)], (100, 100), (900, 100), 350),
        ("offset r=15 below", [(400, 140, 15)], (100, 100), (900, 100), 350),
        ("three staggered", [(300, 60, 10), (400, 140, 10), (600, 100, 10)], (100, 100), (900, 100), 600),
        ("two above+below", [(400, 130, 15), (400, 70, 15)], (100, 100), (900, 100), 600),
        ("8 wall cluster", [
            (400, 60, 12), (400, 140, 12), (500, 80, 12), (500, 120, 12),
            (600, 100, 15), (650, 70, 10), (650, 130, 10), (400, 100, 20),
        ], (100, 100), (900, 100), 1200),
    ])
    def test_navigates_obstacles(self, desc, obstacles, start, goal, max_frames):
        e = SimEngine()
        e.set_goal(start={"x": start[0], "y": start[1]},
                   end={"x": goal[0], "y": goal[1]})
        for ox, oy, r in obstacles:
            e.add_obstacle(ox, oy, r)
        e.set_autopilot(True)
        for _ in range(max_frames):
            s = e.step(0.05)
            if s["arrived"]:
                break
        assert s["arrived"], f"{desc}: did not arrive in {max_frames} frames"
        assert abs(s["x_cm"] - goal[0]) < 30, f"{desc}: x={s['x_cm']:.0f} far from goal {goal[0]}"

    @pytest.mark.parametrize("desc,obstacles,start,goal,max_frames", [
        ("dense 16 left+right clusters", [
            (641, 125, 8), (641, 92, 9), (642, 82, 13), (644, 111, 10),
            (646, 10, 8), (646, 52, 10), (651, 42, 5), (651, 101, 14),
            (733, 77, 8), (733, 186, 11), (735, 97, 9), (737, 169, 9),
            (740, 100, 13), (740, 172, 12), (745, 129, 12), (749, 152, 15),
        ], (500, 100), (900, 100), 2500),
        ("dense 16 start-to-end", [
            (641, 125, 8), (641, 92, 9), (642, 82, 13), (644, 111, 10),
            (646, 10, 8), (646, 52, 10), (651, 42, 5), (651, 101, 14),
            (733, 77, 8), (733, 186, 11), (735, 97, 9), (737, 169, 9),
            (740, 100, 13), (740, 172, 12), (745, 129, 12), (749, 152, 15),
        ], (100, 100), (900, 100), 3000),
    ])
    def test_navigates_dense_clusters(self, desc, obstacles, start, goal, max_frames):
        e = SimEngine()
        e.set_goal(start={"x": start[0], "y": start[1]},
                   end={"x": goal[0], "y": goal[1]})
        for ox, oy, r in obstacles:
            e.add_obstacle(ox, oy, r)
        e.set_autopilot(True)
        for _ in range(max_frames):
            s = e.step(0.05)
            if s["arrived"]:
                break
        assert s["arrived"], f"{desc}: did not arrive in {max_frames} frames"
        assert abs(s["x_cm"] - goal[0]) < 30

    def test_avoid_state_sequence_exists(self):
        e = SimEngine()
        e.set_goal(start={"x": 100, "y": 100}, end={"x": 900, "y": 100})
        e.add_obstacle(400, 100, 30)
        e.set_autopilot(True)

        states_seen = set()
        for _ in range(500):
            s = e.step(0.05)
            states_seen.add(s["brain_state"])
            if s["arrived"]:
                break

        assert "REVERSE" in states_seen
        assert "TURN" in states_seen
        assert "DRIVE" in states_seen
        assert "ARRIVED" in states_seen

    def test_perim_escape_works(self):
        """Wall cluster pushes duck to perim — perim should recover, not trap."""
        e = SimEngine()
        e.PH = 200
        e.set_goal(start={"x": 500, "y": 100}, end={"x": 900, "y": 100})
        # Wall-like obstacle blocking path at top-right
        for ox, oy, r in [
            (700, 170, 15), (700, 185, 10), (720, 175, 12), (740, 180, 10),
            (760, 170, 8), (780, 175, 10),
        ]:
            e.add_obstacle(ox, oy, r)
        e.set_autopilot(True)
        perim_seen = False
        for _ in range(1500):
            s = e.step(0.05)
            if s["brain_state"] == "PERIM":
                perim_seen = True
            if s["arrived"]:
                break
        assert s["arrived"], "should arrive past perim traps"

    def test_load_scenario(self):
        e = SimEngine()
        e.load_scenario({
            "config": {"max_speed": 0.5},
            "obstacles": [{"x": 300, "y": 80, "r": 15}, {"x": 700, "y": 120, "r": 20}],
            "start": {"x": 100, "y": 100},
            "end": {"x": 900, "y": 100},
        })
        s = e.get_state()
        assert len(s["obstacles"]) == 2
        assert s["start"]["x"] == 100
        assert s["end"]["x"] == 900
        assert s["x_cm"] == 100

    def test_mline_on_line(self):
        e = SimEngine()
        e.set_goal(start={"x": 100, "y": 100}, end={"x": 900, "y": 100})
        assert e._on_mline(500, 100)        # center of m-line
        assert e._on_mline(200, 98)         # near start, with slight offset
        assert not e._on_mline(500, 60)     # far off m-line
        assert not e._on_mline(0, 100)      # before start
        assert not e._on_mline(950, 100)    # after end

    def test_mline_diagonal(self):
        e = SimEngine()
        e.set_goal(start={"x": 100, "y": 50}, end={"x": 900, "y": 150})
        assert e._on_mline(500, 100)         # midpoint of diagonal
        assert not e._on_mline(100, 100)     # perpendicular offset at start

    def test_mline_hit_point_recorded(self):
        e = SimEngine()
        e.set_goal(start={"x": 100, "y": 100}, end={"x": 900, "y": 100})
        e.set_pose(300, 100, 0)
        e.set_sonar_override(front=20, left=9999, right=9999)
        e.set_autopilot(True)
        s = e.step(0.05)
        assert s["avoid_state"] == "reverse"
        assert e._mline_hit_x == 300
        assert e._mline_hit_y == 100
        assert e._mline_hit_dist > 500

    def test_bug2_exits_past_obstacle(self):
        """Verify DRIVE exits when duck crosses m-line closer to goal than hit point."""
        e = SimEngine()
        e.set_goal(start={"x": 100, "y": 100}, end={"x": 900, "y": 100})
        e.add_obstacle(400, 100, 30)
        e.set_autopilot(True)
        for _ in range(3000):
            s = e.step(0.05)
            if s["arrived"]:
                break
        assert s["arrived"]
        assert abs(s["x_cm"] - 900) < 30
        assert abs(s["y_cm"] - 100) < 30

    def test_no_jps_references(self):
        e = SimEngine()
        s = e.get_state()
        assert "jps_path" not in s
        assert "jps_path_len" not in s
        assert "replan_count" not in s

    def test_arrives_at_goal_no_obstacles(self):
        e = SimEngine()
        e.set_goal(start={"x": 100, "y": 100}, end={"x": 900, "y": 100})
        e.set_autopilot(True)
        for _ in range(500):
            s = e.step(0.05)
            if s["arrived"]:
                break
        assert s["arrived"]
        assert abs(s["x_cm"] - 900) < 30

    def test_step_manual_mode(self):
        e = SimEngine()
        e.set_speeds(0.4, 0.6)
        prev_x = e.get_state()["x_cm"]
        e.step(0.05)
        s = e.get_state()
        assert s["x_cm"] != prev_x

    def test_wall_map_visible_by_default(self):
        e = SimEngine()
        assert e.wall_map_visible is True
