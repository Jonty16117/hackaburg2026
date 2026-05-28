"""Finite state machine: sonar + odometry + perimeter → motor speed commands."""

import math
import random
import time as _time
from enum import Enum, auto


class State(Enum):
    EXPLORE = auto()
    AVOID = auto()
    TURN_TO_CENTER = auto()
    STUCK = auto()


class _AvoidPhase(Enum):
    REVERSE = auto()
    SCAN_LEFT = auto()
    SCAN_LEFT_READ = auto()
    SCAN_RIGHT = auto()
    SCAN_RIGHT_READ = auto()
    COMPLETE_TURN = auto()
    DONE = auto()


class Brain:
    def __init__(self, perimeter, cfg):
        self.perimeter = perimeter
        self.cfg = cfg

        self.state = State.EXPLORE
        self._state_start = _time.time()

        self._avoid_phase = _AvoidPhase.DONE
        self._avoid_phase_start = 0.0
        self._d_left = None
        self._d_right = None
        self._chosen_dir = None
        self._avoid_cooldown = 0.0
        self._avoid_history = []

        self._explore_jitter_timer = 0.0
        self._explore_jitter_bias = 0.0

        self._turn_alternate = 0
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
            self._d_left = None
            self._d_right = None
            self._chosen_dir = None
            now = _time.time()
            self._avoid_history.append(now)
            cutoff = now - self.cfg["STUCK_WINDOW_TIME"]
            self._avoid_history = [t for t in self._avoid_history if t > cutoff]

    def _pick_direction(self, d_left, d_right):
        if d_left is not None and d_right is not None:
            return "left" if d_left > d_right else "right"
        if d_left is not None:
            return "left"
        if d_right is not None:
            return "right"
        self._turn_alternate += 1
        return "left" if self._turn_alternate % 2 == 0 else "right"

    def _heading_error(self, target, current):
        error = target - current
        return ((error + math.pi) % (2 * math.pi)) - math.pi

    def decide(self, sonar_cm, x, y, theta, dt):
        if sonar_cm is not None:
            self._sonar_fail_count = 0
        else:
            self._sonar_fail_count += 1

        if self.state == State.STUCK:
            return self._handle_stuck()

        if self.state == State.AVOID:
            return self._handle_avoid(sonar_cm)

        if (
            self._avoid_cooldown <= 0
            and sonar_cm is not None
            and sonar_cm < self.cfg["OBSTACLE_THRESHOLD_CM"]
        ):
            if len(self._avoid_history) >= self.cfg["STUCK_THRESHOLD_COUNT"]:
                self._transition(State.STUCK)
                return self._handle_stuck()
            self._transition(State.AVOID)
            return self._handle_avoid(sonar_cm)

        if not self.perimeter.is_inside(x, y) or self.perimeter.is_near_edge(
            x, y, self.cfg["PERIMETER_MARGIN_CM"]
        ):
            self._transition(State.TURN_TO_CENTER)
            return self._handle_turn_to_center(x, y, theta)

        if self.state == State.TURN_TO_CENTER:
            return self._handle_turn_to_center(x, y, theta)

        if self._avoid_cooldown > 0:
            self._avoid_cooldown = max(0, self._avoid_cooldown - dt)

        return self._handle_explore(dt)

    def _handle_explore(self, dt):
        self._explore_jitter_timer += dt
        if self._explore_jitter_timer > self.cfg["EXPLORE_JITTER_TIME"]:
            self._explore_jitter_timer = 0
            self._explore_jitter_bias = random.uniform(
                -self.cfg["EXPLORE_JITTER_AMOUNT"],
                self.cfg["EXPLORE_JITTER_AMOUNT"],
            )

        speed = self.cfg["EXPLORE_SPEED"]
        if self._sonar_fail_count > self.cfg["SONAR_FAIL_THRESHOLD"]:
            speed *= self.cfg["SONAR_FAIL_SPEED_SCALE"]

        left_speed = speed - self._explore_jitter_bias
        right_speed = speed + self._explore_jitter_bias
        return left_speed, right_speed

    def _handle_avoid(self, sonar_cm):
        elapsed = self.avoid_phase_timer

        if self._avoid_phase == _AvoidPhase.REVERSE:
            if elapsed >= self.cfg["AVOID_REVERSE_TIME"]:
                self._avoid_phase = _AvoidPhase.SCAN_LEFT
                self._avoid_phase_start = _time.time()
                return self._handle_avoid(sonar_cm)
            return (-self.cfg["AVOID_REVERSE_SPEED"], -self.cfg["AVOID_REVERSE_SPEED"])

        if self._avoid_phase == _AvoidPhase.SCAN_LEFT:
            if elapsed >= self.cfg["SCAN_LEFT_TIME"]:
                self._avoid_phase = _AvoidPhase.SCAN_LEFT_READ
                self._avoid_phase_start = _time.time()
                return self._handle_avoid(sonar_cm)
            return (-self.cfg["SCAN_SPEED"], self.cfg["SCAN_SPEED"])

        if self._avoid_phase == _AvoidPhase.SCAN_LEFT_READ:
            if self._d_left is None and sonar_cm is not None:
                self._d_left = sonar_cm
            if elapsed >= self.cfg["SCAN_READ_PAUSE"]:
                self._avoid_phase = _AvoidPhase.SCAN_RIGHT
                self._avoid_phase_start = _time.time()
                return self._handle_avoid(sonar_cm)
            return (0.0, 0.0)

        if self._avoid_phase == _AvoidPhase.SCAN_RIGHT:
            if elapsed >= self.cfg["SCAN_RIGHT_TIME"]:
                self._avoid_phase = _AvoidPhase.SCAN_RIGHT_READ
                self._avoid_phase_start = _time.time()
                return self._handle_avoid(sonar_cm)
            return (self.cfg["SCAN_SPEED"], -self.cfg["SCAN_SPEED"])

        if self._avoid_phase == _AvoidPhase.SCAN_RIGHT_READ:
            if self._d_right is None and sonar_cm is not None:
                self._d_right = sonar_cm
            if elapsed >= self.cfg["SCAN_READ_PAUSE"]:
                self._chosen_dir = self._pick_direction(self._d_left, self._d_right)
                self._avoid_phase = _AvoidPhase.COMPLETE_TURN
                self._avoid_phase_start = _time.time()
                return self._handle_avoid(sonar_cm)
            return (0.0, 0.0)

        if self._avoid_phase == _AvoidPhase.COMPLETE_TURN:
            if elapsed >= self.cfg["COMPLETE_TURN_TIME"]:
                self._avoid_phase = _AvoidPhase.DONE
                self._avoid_phase_start = _time.time()
                return self._handle_avoid(sonar_cm)
            if self._chosen_dir == "left":
                return (
                    -self.cfg["COMPLETE_TURN_SPEED"],
                    self.cfg["COMPLETE_TURN_SPEED"],
                )
            return (
                self.cfg["COMPLETE_TURN_SPEED"],
                -self.cfg["COMPLETE_TURN_SPEED"],
            )

        self._transition(State.EXPLORE)
        self._avoid_cooldown = self.cfg["AVOID_COOLDOWN_TIME"]
        return self.cfg["EXPLORE_SPEED"], self.cfg["EXPLORE_SPEED"]

    def _handle_turn_to_center(self, x, y, theta):
        bearing = self.perimeter.bearing_to_center(x, y)
        error = self._heading_error(bearing, theta)

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
