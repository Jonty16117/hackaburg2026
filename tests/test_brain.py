from navigation.brain import Brain, State, _AvoidPhase
from navigation.perimeter import Perimeter
from navigation.config import END_X_CM, END_Y_CM


PERIMETER_CM = [
    (0, 0),
    (1000, 0),
    (1000, 200),
    (0, 200),
]

BRAIN_CFG = {
    "PERIMETER_MARGIN_CM": 30,
    "EXPLORE_SPEED": 0.4,
    "TURN_SPEED": 0.5,
    "AVOID_REVERSE_SPEED": 0.5,
    "SCAN_SPEED": 0.4,
    "AVOID_REVERSE_TIME": 0.3,
    "AVOID_COOLDOWN_TIME": 0.5,
    "AVOID_CLEAR_THRESHOLD_CM": 100,
    "AVOID_MAX_SCAN_RAD": 1.57,
    "AVOID_REACTIVE_KP": 0.003,
    "AVOID_TARGET_DIST_CM": 30,
    "AVOID_REACTIVE_FWD_SPEED": 0.3,
    "AVOID_REACTIVE_TIMEOUT": 30.0,
    "GOAL_X": END_X_CM,
    "GOAL_Y": END_Y_CM,
    "STUCK_THRESHOLD_COUNT": 3,
    "STUCK_WINDOW_TIME": 10.0,
    "STUCK_ESCAPE_TIME": 2.0,
    "MAX_SPEED_CM_S": 100,
    "WHEEL_BASE_CM": 30,
    "HEADING_TOLERANCE_RAD": 0.26,
    "EXPLORE_JITTER_TIME": 6.0,
    "EXPLORE_JITTER_AMOUNT": 0.10,
    "SONAR_FAIL_THRESHOLD": 5,
    "SONAR_FAIL_SPEED_SCALE": 0.3,
    "LOOP_HZ": 20,
    "OBSTACLE_THRESHOLD_CM": 50,
}


def _brain():
    p = Perimeter(PERIMETER_CM)
    return Brain(p, BRAIN_CFG, goal_x=END_X_CM, goal_y=END_Y_CM)


def test_initial_state():
    b = _brain()
    assert b.state == State.EXPLORE


def test_explore_drives_forward():
    b = _brain()
    ls, rs = b.decide(500, 500, 100, 0, 0.05)
    assert ls > 0
    assert rs > 0


def test_obstacle_triggers_avoid():
    b = _brain()
    ls, rs = b.decide(20, 500, 100, 0, 0.05)
    assert b.state == State.AVOID
    assert ls < 0
    assert rs < 0


def test_avoid_progresses_through_phases():
    b = _brain()
    assert b._avoid_phase == _AvoidPhase.REVERSE

    b.decide(20, 500, 100, 0, 0.05)
    assert b.state == State.AVOID
    assert b._avoid_phase == _AvoidPhase.REVERSE

    # TURN_AND_SENSE after sufficient real time
    import time
    time.sleep(BRAIN_CFG["AVOID_REVERSE_TIME"] + 0.1)
    b.decide(20, 500, 100, 0, 0.05)
    assert b._avoid_phase != _AvoidPhase.REVERSE


def test_avoid_cooldown_blocks_retrigger():
    b = _brain()
    b.decide(20, 500, 100, 0, 0.05)
    assert b.state == State.AVOID

    b._avoid_cooldown = 1.0
    b.state = State.EXPLORE
    ls, rs = b.decide(20, 500, 100, 0, 0.05)
    assert b.state == State.EXPLORE


def test_perimeter_triggers_turn_to_center():
    b = _brain()
    ls, rs = b.decide(500, 0, 0, 0, 0.05)
    assert b.state == State.TURN_TO_CENTER


def test_perimeter_near_edge_triggers_turn_to_center():
    b = _brain()
    ls, rs = b.decide(500, 5, 100, 0, 0.05)
    assert b.state == State.TURN_TO_CENTER


def test_stuck_detection():
    b = _brain()
    for _ in range(BRAIN_CFG["STUCK_THRESHOLD_COUNT"]):
        b._avoid_history.append(0)
    ls, rs = b.decide(20, 500, 100, 0, 0.05)
    assert b.state == State.STUCK


def test_stuck_escapes_after_time():
    import time
    b = _brain()
    b.state = State.STUCK
    b._state_start = time.time() - BRAIN_CFG["STUCK_ESCAPE_TIME"] - 1
    ls, rs = b._handle_stuck()
    assert b.state == State.EXPLORE


def test_sonar_failure_reduces_speed():
    b = _brain()
    b._sonar_fail_count = BRAIN_CFG["SONAR_FAIL_THRESHOLD"] + 1
    ls, rs = b._handle_explore(0.05)
    assert abs(ls) < BRAIN_CFG["EXPLORE_SPEED"]


def test_explore_has_jitter():
    import time
    b = _brain()
    b._explore_jitter_timer = BRAIN_CFG["EXPLORE_JITTER_TIME"] + 1
    initial_bias = b._explore_jitter_bias
    ls, rs = b._handle_explore(0.05)
    assert ls != rs
