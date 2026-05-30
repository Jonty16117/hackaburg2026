"""Finite state machine: sonar + odometry + perimeter → motor speed commands."""

import math
import random
import time as _time
from enum import Enum, auto
from navigation.utils import heading_error


class State(Enum):
    EXPLORE = auto()
    AVOID = auto()
    TURN_TO_CENTER = auto()
    STUCK = auto()


class _AvoidPhase(Enum):
    REVERSE = auto()
    TURN_AND_SENSE = auto()
    FACE_OPENING = auto()
    REACTIVE_DRIVE = auto()


def _fuse_sonar(sl, sf, sr):
    valid = [v for v in (sl, sf, sr) if v is not None]
    return min(valid) if valid else None


class Brain:
    def __init__(self, perimeter, cfg, goal_x=None, goal_y=None):
        self.perimeter = perimeter
        self.cfg = cfg
        self.goal_x = goal_x if goal_x is not None else cfg.get("GOAL_X")
        self.goal_y = goal_y if goal_y is not None else cfg.get("GOAL_Y")

        self.state = State.EXPLORE
        self._state_start = _time.time()

        self._avoid_phase = _AvoidPhase.REVERSE
        self._avoid_phase_start = 0.0
        self._avoid_cooldown = 0.0
        self._avoid_history = []
        self._best_heading = None
        self._max_sonar_seen = 0.0
        self._avoid_turn_dir = 1
        self._reactive_start = 0.0

        self._explore_jitter_timer = 0.0
        self._explore_jitter_bias = 0.0

        self._sonar_fail_count = 0

    @property
    def state_timer(self):
        return _time.time() - self._state_start

    @property
    def avoid_phase_timer(self):
        return _time.time() - self._avoid_phase_start

    def _transition(self, new_state):
        self.state = new_state
        self._state_start = _time.time()
        if new_state == State.AVOID:
            self._avoid_phase = _AvoidPhase.REVERSE
            self._avoid_phase_start = _time.time()
            self._best_heading = None
            self._max_sonar_seen = 0.0
            self._avoid_turn_dir = -self._avoid_turn_dir
            self._reactive_start = 0.0
            now = _time.time()
            self._avoid_history.append(now)
            cutoff = now - self.cfg["STUCK_WINDOW_TIME"]
            self._avoid_history = [t for t in self._avoid_history if t > cutoff]

    def decide(self, sl, sf, sr, x, y, theta, dt):
        sonar_cm = _fuse_sonar(sl, sf, sr)
        if sonar_cm is not None:
            self._sonar_fail_count = 0
        else:
            self._sonar_fail_count += 1

        if self.state == State.STUCK:
            return self._handle_stuck()

        if self.state == State.AVOID:
            return self._handle_avoid(sonar_cm, sl, sf, sr, x, y, theta, dt)

        if self._avoid_cooldown > 0:
            self._avoid_cooldown = max(0, self._avoid_cooldown - dt)

        if (
            self._avoid_cooldown <= 0
            and sonar_cm is not None
            and sonar_cm < self.cfg["OBSTACLE_THRESHOLD_CM"]
        ):
            if len(self._avoid_history) >= self.cfg["STUCK_THRESHOLD_COUNT"]:
                self._transition(State.STUCK)
                return self._handle_stuck()
            self._transition(State.AVOID)
            return self._handle_avoid(sonar_cm, sl, sf, sr, x, y, theta, dt)

        if not self.perimeter.is_inside(x, y) or self.perimeter.is_near_edge(
            x, y, self.cfg["PERIMETER_MARGIN_CM"]
        ):
            self._transition(State.TURN_TO_CENTER)
            return self._handle_turn_to_center(x, y, theta)

        if self.state == State.TURN_TO_CENTER:
            return self._handle_turn_to_center(x, y, theta)

        return self._handle_explore(dt, sl, sf, sr)

    def _handle_explore(self, dt, sl, sf, sr):
        self._explore_jitter_timer += dt
        if self._explore_jitter_timer > self.cfg["EXPLORE_JITTER_TIME"]:
            self._explore_jitter_timer = 0
            self._explore_jitter_bias = random.uniform(
                -self.cfg["EXPLORE_JITTER_AMOUNT"],
                self.cfg["EXPLORE_JITTER_AMOUNT"],
            )

        bias = self._explore_jitter_bias
        speed = self.cfg["EXPLORE_SPEED"]

        # Pre-emptive steering: use side sonars to bias away from obstacles
        if (sf is not None and sl is not None and sr is not None
                and sf < self.cfg["LOOK_AHEAD_CM"]):
            if sl > sr + 30:
                bias = self.cfg["EXPLORE_JITTER_AMOUNT"] * 2
            elif sr > sl + 30:
                bias = -self.cfg["EXPLORE_JITTER_AMOUNT"] * 2
            else:
                speed *= 0.5

        if self._sonar_fail_count > self.cfg["SONAR_FAIL_THRESHOLD"]:
            speed *= self.cfg["SONAR_FAIL_SPEED_SCALE"]

        return speed - bias, speed + bias

    def _handle_avoid(self, sonar_cm, sl, sf, sr, x, y, theta, dt):
        elapsed = self.avoid_phase_timer
        now = _time.time()

        # ── Phase 1: PAUSE (was REVERSE) ──
        if self._avoid_phase == _AvoidPhase.REVERSE:
            if elapsed >= self.cfg["AVOID_REVERSE_TIME"]:
                self._avoid_phase = _AvoidPhase.TURN_AND_SENSE
                self._avoid_phase_start = now
            return (0.0, 0.0)

        # ── Phase 2: TURN_AND_SENSE ──
        if self._avoid_phase == _AvoidPhase.TURN_AND_SENSE:
            if sonar_cm is not None and sonar_cm > self._max_sonar_seen:
                self._max_sonar_seen = sonar_cm
                self._best_heading = theta

            if self._max_sonar_seen >= self.cfg["AVOID_CLEAR_THRESHOLD_CM"]:
                self._avoid_phase = _AvoidPhase.FACE_OPENING
                self._avoid_phase_start = now

            turned = (self.cfg["SCAN_SPEED"] * self.cfg["MAX_SPEED_CM_S"]
                      / self.cfg["WHEEL_BASE_CM"] * elapsed)
            if turned >= self.cfg["AVOID_MAX_SCAN_RAD"]:
                if self._max_sonar_seen < self.cfg["AVOID_CLEAR_THRESHOLD_CM"]:
                    self._transition(State.STUCK)
                    return self._handle_stuck()
                self._avoid_phase = _AvoidPhase.FACE_OPENING
                self._avoid_phase_start = now

            sd = self._avoid_turn_dir
            return (-self.cfg["SCAN_SPEED"] * sd, self.cfg["SCAN_SPEED"] * sd)

        # ── Phase 3: FACE_OPENING ──
        if self._avoid_phase == _AvoidPhase.FACE_OPENING:
            target = self._best_heading if self._best_heading is not None else theta
            err = heading_error(target, theta)
            if abs(err) < self.cfg["HEADING_TOLERANCE_RAD"]:
                self._avoid_phase = _AvoidPhase.REACTIVE_DRIVE
                self._avoid_phase_start = now
                self._reactive_start = now
            td = 1 if err > 0 else -1
            return (-self.cfg["TURN_SPEED"] * td, self.cfg["TURN_SPEED"] * td)

        # ── Phase 4: REACTIVE_DRIVE ──
        if self._reactive_start == 0:
            self._reactive_start = now
        reactive_elapsed = now - self._reactive_start

        # Obstacle closing in again — re-scan immediately
        if (sonar_cm is not None
                and sonar_cm < self.cfg["OBSTACLE_THRESHOLD_CM"]):
            self._avoid_phase = _AvoidPhase.TURN_AND_SENSE
            self._avoid_phase_start = now
            self._max_sonar_seen = 0.0
            self._best_heading = None
            self._reactive_start = 0.0
            return (0.0, 0.0)

        # Exit: clear path ahead and roughly facing goal
        if (sonar_cm is not None
                and sonar_cm > self.cfg["AVOID_CLEAR_THRESHOLD_CM"]
                and self.goal_x is not None and self.goal_y is not None):
            goal_th = math.atan2(self.goal_y - y, self.goal_x - x)
            err = heading_error(goal_th, theta)
            if abs(err) < self.cfg["HEADING_TOLERANCE_RAD"] * 2:
                self._transition(State.EXPLORE)
                self._avoid_cooldown = self.cfg["AVOID_COOLDOWN_TIME"]
                return self._handle_explore(dt, sl, sf, sr)

        # Timeout fallback: drive burst over, re-scan
        if reactive_elapsed >= self.cfg["AVOID_REACTIVE_TIMEOUT"]:
            self._avoid_phase = _AvoidPhase.TURN_AND_SENSE
            self._avoid_phase_start = now
            self._max_sonar_seen = 0.0
            self._best_heading = None
            self._reactive_start = 0.0
            return (0.0, 0.0)

        # Straight drive forward (I2C has no variable speed)
        fwd = self.cfg["AVOID_REACTIVE_FWD_SPEED"]
        return (fwd, fwd)

    def _handle_turn_to_center(self, x, y, theta):
        bearing = self.perimeter.bearing_to_center(x, y)
        error = heading_error(bearing, theta)

        if abs(error) < self.cfg["HEADING_TOLERANCE_RAD"]:
            if not self.perimeter.is_near_edge(
                x, y, self.cfg["PERIMETER_MARGIN_CM"]
            ):
                self._transition(State.EXPLORE)
                return self.cfg["EXPLORE_SPEED"], self.cfg["EXPLORE_SPEED"]
            return self.cfg["EXPLORE_SPEED"], self.cfg["EXPLORE_SPEED"]

        turn_dir = 1 if error > 0 else -1
        ts = self.cfg["TURN_SPEED"]
        return (-ts * turn_dir, ts * turn_dir)

    def _handle_stuck(self):
        if self.state_timer > self.cfg["STUCK_ESCAPE_TIME"]:
            self._avoid_history.clear()
            self._transition(State.EXPLORE)
            return self.cfg["EXPLORE_SPEED"], self.cfg["EXPLORE_SPEED"]
        return (-self.cfg["TURN_SPEED"], self.cfg["TURN_SPEED"])
