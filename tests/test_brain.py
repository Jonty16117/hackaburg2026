from navigation.brain import Brain, State, _AvoidPhase
from navigation.perimeter import Perimeter
from navigation.config import PERIMETER_CM, BRAIN_CFG, END_X_CM, END_Y_CM


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
    assert ls == 0
    assert rs == 0


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
