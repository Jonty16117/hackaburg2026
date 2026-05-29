"""Server-side simulation engine for DuckBot.

Ports the browser JS physics + autopilot FSM to Python so agents
can drive the simulation programmatically via REST / WebSocket.

Usage:
    from dashboard.sim_engine import SimEngine
    engine = SimEngine()
    engine.step(0.05)
    print(engine.get_state())
"""

import math
import time
import threading

from navigation.utils import clamp, normalize_angle, heading_error
from navigation.wall_map import WallMap
from navigation.sonar_sweep import simulate_sweep


class SimEngine:
    """Port of the browser duck simulation — physics + autopilot."""

    def __init__(self):
        self._lock = threading.Lock()

        # --- geometry ---
        self.PW = 1000
        self.PH = 200
        self.DUCK_R = 15
        self.MARGIN = 20
        self.ARRIVAL = 20

        # --- autopilot config (mutable) ---
        self.OBST_TH = 25
        self.TURN_SP = 0.5
        self.REVERSE_SPD = 0.5
        self.MAX_SPD = 100
        self.WB = 30
        self.HDG_TOL = 0.02
        self.AVOID_REVERSE_S = 0.5
        self.AVOID_COOLDOWN_S = 1.5
        self.AVOID_REACTIVE_FWD_SPEED = 0.5
        self.AVOID_REACTIVE_TIMEOUT = 10.0
        self.AVOID_REACTIVE_MIN_TIME = 2.0
        self.AVOID_TARGET_DIST_CM = 30
        self.AVOID_CLEAR_THRESHOLD_CM = 100
        self.STUCK_DIST_CM = 8
        self.STUCK_WINDOW_S = 1.5
        self.max_speed = 0.8

        # --- duck state ---
        self.x = 500.0
        self.y = 100.0
        self.theta = 0.0
        self.left_speed = 0.0
        self.right_speed = 0.0
        self._blocked_count = 0

        # --- sensors ---
        self.sonar_front = None
        self.sonar_left = None
        self.sonar_right = None
        self._sonar_override = {"front": None, "left": None, "right": None}

        # --- environment ---
        self.obstacles = []    # [{id, x, y, r}]
        self._next_obs_id = 0
        self.start = {"x": 500, "y": 100}
        self.end = {"x": 900, "y": 100}

        # --- wall mapping ---
        self.wall_map = None
        self.wall_map_visible = False
        self.sweep_requested = False
        self.sweep_readings = None

        # --- autopilot FSM ---
        self.autopilot_on = False
        self.arrived = False
        self.avoid_state = "none"       # none | reverse | turn | drive
        self.avoid_timer = 0.0
        self._avoid_cooldown = 0.0
        self._avoid_goal_check = 0.0    # when to check if path to goal is clear
        self._avoid_turn_dir = 1        # alternating turn direction
        self._turn_target_h = 0.0
        self._turn_rescored = False
        self._avoid_cycles = 0          # rapid DRIVE→REVERSE count
        self._reactive_start_x = 0.0
        self._reactive_start_y = 0.0
        self._mline_hit_x = 0.0
        self._mline_hit_y = 0.0
        self._mline_hit_dist = float("inf")
        self.pos_history = []
        self.perim_escaping = False
        self.escape_heading = 0.0
        self.perim_cooldown = 0.0
        self._perim_timer = 0.0

        # --- debug & sim time ---
        self.frame = 0
        self.sim_time = 0.0
        self.trail = []   # last 40 pos
        self.debug_buf = []  # last 500 frames
        self._start_time = time.time()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _nose_position(self):
        return {
            "x": self.x + 14 * math.cos(self.theta),
            "y": self.y + 14 * math.sin(self.theta),
        }

    # --- raycast (port of JS ray) ---
    def _raycast(self, ox, oy, angle):
        best = float("inf")
        dx = math.cos(angle)
        dy = math.sin(angle)

        def _hit(x3, y3, x4, y4):
            nonlocal best
            d = dx * (y3 - y4) - dy * (x3 - x4)
            if abs(d) < 1e-9:
                return
            t = ((ox - x3) * (y3 - y4) - (oy - y3) * (x3 - x4)) / d
            u = -((dx * (oy - y3) - dy * (ox - x3))) / d
            # Note: the formula computes t = -t_standard (denominator sign inversion)
            # but u = u_standard (correct). So check t <= 0 for "in front" and 0 <= u <= 1 for "within segment".
            if t <= 0 and 0 <= u <= 1:
                dist = math.hypot(ox + dx * t - ox, oy + dy * t - oy)
                if dist < best:
                    best = dist

        # perimeter walls
        _hit(0, 0, self.PW, 0)
        _hit(self.PW, 0, self.PW, self.PH)
        _hit(self.PW, self.PH, 0, self.PH)
        _hit(0, self.PH, 0, 0)

        # obstacles (circles)
        for o in self.obstacles:
            fx = ox - o["x"]
            fy = oy - o["y"]
            a = dx * dx + dy * dy
            b = 2 * (fx * dx + fy * dy)
            c = fx * fx + fy * fy - o["r"] * o["r"]
            disc = b * b - 4 * a * c
            if disc < 0:
                continue
            sqrt_disc = math.sqrt(disc)
            t1 = (-b - sqrt_disc) / (2 * a)
            t2 = (-b + sqrt_disc) / (2 * a)
            if t1 >= 0 and t1 < best:
                best = t1
            elif t2 >= 0 and t2 < best:
                best = t2

        return best if best < float("inf") else None

    # --- physics step (port of JS step) ---
    def _step_physics(self, dt):
        vl = self.left_speed * self.MAX_SPD
        vr = self.right_speed * self.MAX_SPD
        v = (vl + vr) / 2.0
        w = (vr - vl) / self.WB

        nx = self.x + v * math.cos(self.theta) * dt
        ny = self.y + v * math.sin(self.theta) * dt
        nx = clamp(nx, self.DUCK_R, self.PW - self.DUCK_R)
        ny = clamp(ny, self.DUCK_R, self.PH - self.DUCK_R)

        if not self._obstacle_at(nx, ny):
            self.x = nx
            self.y = ny

        self.theta += w * dt
        self.theta = normalize_angle(self.theta)

    def _obstacle_at(self, x, y):
        for o in self.obstacles:
            if math.hypot(x - o["x"], y - o["y"]) < self.DUCK_R + o["r"]:
                return True
        return False

    def _on_mline(self, x, y):
        dx = self.end["x"] - self.start["x"]
        dy = self.end["y"] - self.start["y"]
        ll = dx * dx + dy * dy
        if ll < 1:
            return False
        t = ((x - self.start["x"]) * dx + (y - self.start["y"]) * dy) / ll
        if not (0.05 <= t <= 0.95):
            return False
        px = self.start["x"] + t * dx
        py = self.start["y"] + t * dy
        return math.hypot(x - px, y - py) < 15

    # --- autopilot FSM (port of JS autoPilot) ---
    def _autopilot(self, dt):
        dist_end = math.hypot(self.x - self.end["x"], self.y - self.end["y"])
        if dist_end < self.ARRIVAL:
            self.arrived = True
            return 0.0, 0.0
        if self.arrived:
            self.arrived = False

        f = self.sonar_front if self.sonar_front is not None else 9999
        obs_dist = self.OBST_TH + self.DUCK_R

        goal_th = math.atan2(self.end["y"] - self.y, self.end["x"] - self.x)
        goal_err = heading_error(goal_th, self.theta)

        # Stuck detection (GO phase)
        if self.avoid_state == "none" and not self.perim_escaping:
            self.pos_history.append({"x": self.x, "y": self.y, "t": self.sim_time})
            cutoff = self.sim_time - self.STUCK_WINDOW_S
            self.pos_history = [p for p in self.pos_history if p["t"] >= cutoff]
            stuck = False
            if len(self.pos_history) >= 10:
                mid = len(self.pos_history) // 2
                p_mid = self.pos_history[mid]
                p_n = self.pos_history[-1]
                if (p_n["t"] - p_mid["t"] >= self.STUCK_WINDOW_S / 2
                        and math.hypot(p_n["x"] - p_mid["x"], p_n["y"] - p_mid["y"]) < self.STUCK_DIST_CM):
                    stuck = True
            if stuck:
                self.avoid_state = "reverse"
                self.avoid_timer = 0.0
                self._mline_hit_x = self.x
                self._mline_hit_y = self.y
                self._mline_hit_dist = math.hypot(self.x - self.end["x"], self.y - self.end["y"])
                self.pos_history = []
                return -self.REVERSE_SPD, -self.REVERSE_SPD

        # Perimeter escape detection
        near = min(self.x, self.PW - self.x, self.y, self.PH - self.y)
        if (near < self.MARGIN and not self.perim_escaping
                and self.perim_cooldown <= 0
                and self.avoid_state == "none"):
            dx = 0.0
            dy = 0.0
            if self.x < self.MARGIN:
                dx = self.MARGIN - self.x
            if self.PW - self.x < self.MARGIN:
                dx -= self.x - (self.PW - self.MARGIN)
            if self.y < self.MARGIN:
                dy = self.MARGIN - self.y
            if self.PH - self.y < self.MARGIN:
                dy -= self.y - (self.PH - self.MARGIN)
            self.escape_heading = math.atan2(dy, dx)
            self.perim_escaping = True
            self._perim_timer = 0.0
            self.avoid_state = "none"

        if near > self.MARGIN * 2 and self.perim_escaping:
            self.perim_escaping = False
            self.perim_cooldown = 0.5
            self._perim_timer = 0.0

        if self.perim_cooldown > 0:
            self.perim_cooldown = max(0, self.perim_cooldown - dt)

        if self.perim_escaping:
            self._perim_timer += dt
            perr = heading_error(self.escape_heading, self.theta)
            if abs(perr) < 0.08:
                return self.max_speed, self.max_speed
            if self._perim_timer > 2.0:
                self.perim_escaping = False
                self.perim_cooldown = 2.0
                self._perim_timer = 0.0
                self.avoid_state = "reverse"
                self.avoid_timer = 0.0
                self._mline_hit_x = self.x
                self._mline_hit_y = self.y
                self._mline_hit_dist = math.hypot(self.x - self.end["x"], self.y - self.end["y"])
                return -self.REVERSE_SPD, -self.REVERSE_SPD
            return -math.copysign(1, perr), math.copysign(1, perr)

        if self.perim_cooldown > 0 and self.avoid_state == "none":
            cperr = heading_error(self.escape_heading, self.theta)
            if abs(cperr) < 0.08:
                return self.max_speed, self.max_speed
            return -math.copysign(1, cperr), math.copysign(1, cperr)

        # Avoid FSM: REVERSE -> TURN -> DRIVE
        if self.avoid_state != "none":
            self.avoid_timer += dt
            if self.avoid_state == "reverse":
                if self.avoid_timer >= self.AVOID_REVERSE_S:
                    self.avoid_state = "turn"
                    self.avoid_timer = 0.0
                    self._turn_rescored = False
                    nx, ny = self._nose_position()["x"], self._nose_position()["y"]
                    goal_th = math.atan2(self.end["y"] - self.y, self.end["x"] - self.x)
                    test_offsets = [-math.pi / 3, -math.pi / 6, 0, math.pi / 6, math.pi / 3]
                    if self._avoid_cycles >= 3:
                        test_offsets = [-math.pi / 2, -math.pi / 3, -math.pi / 6, 0, math.pi / 6, math.pi / 3, math.pi / 2]
                    best_h = None; best_score = -999
                    for off in test_offsets:
                        test_h = self.theta + off
                        rng = self._raycast(nx, ny, test_h) or 9999
                        clearance = min(rng, 200) / 200.0
                        goal_align = (math.cos(heading_error(goal_th, test_h)) + 1) / 2
                        score = clearance * 0.4 + goal_align * 0.6
                        if score > best_score:
                            best_score = score; best_h = test_h
                    self._turn_target_h = best_h
                    self._avoid_turn_dir = 1 if heading_error(best_h, self.theta) >= 0 else -1
                else:
                    return -self.REVERSE_SPD, -self.REVERSE_SPD
            if self.avoid_state == "turn":
                err = heading_error(self._turn_target_h, self.theta)
                td = 1 if err >= 0 else -1
                front_blocked = self.sonar_front is not None and self.sonar_front < self.AVOID_TARGET_DIST_CM
                if abs(err) < 0.08 and not front_blocked:
                    self.avoid_state = "drive"
                    self.avoid_timer = 0.0
                    self._drive_steer_sign = 0
                    self._drive_steer_flips = 0
                    self._drive_flip_time = 0.0
                    self._reactive_start_x = self.x
                    self._reactive_start_y = self.y
                elif abs(err) < 0.08 and front_blocked and not self._turn_rescored:
                    self._turn_rescored = True
                    nx, ny = self._nose_position()["x"], self._nose_position()["y"]
                    goal_th = math.atan2(self.end["y"] - self.y, self.end["x"] - self.x)
                    re_offsets = [-math.pi / 2, -math.pi / 3, -math.pi / 6, 0, math.pi / 6, math.pi / 3, math.pi / 2]
                    best_h2 = None; best_score2 = -999
                    for off in re_offsets:
                        test_h = self.theta + off
                        rng = self._raycast(nx, ny, test_h) or 9999
                        clearance = min(rng, 200) / 200.0
                        goal_align = (math.cos(heading_error(goal_th, test_h)) + 1) / 2
                        score = clearance * 0.4 + goal_align * 0.6
                        if score > best_score2:
                            best_score2 = score; best_h2 = test_h
                    self._turn_target_h = best_h2
                else:
                    can_drive = False
                    if self.avoid_timer >= 0.15 and self.sonar_front is not None and self.sonar_front > self.AVOID_CLEAR_THRESHOLD_CM:
                        can_drive = True
                    elif self.avoid_timer >= 0.25 and not front_blocked:
                        can_drive = True
                    elif self.avoid_timer >= 0.5:
                        can_drive = True
                    if can_drive:
                        self.avoid_state = "drive"
                        self.avoid_timer = 0.0
                        self._drive_steer_sign = 0
                        self._drive_steer_flips = 0
                        self._drive_flip_time = 0.0
                        self._reactive_start_x = self.x
                        self._reactive_start_y = self.y
                        self._turn_rescored = False
                    else:
                        return (-self.TURN_SP * td, self.TURN_SP * td)
            if self.avoid_state == "drive":
                # Bug2 exit: crossed m-line closer to goal than hit point
                if self.avoid_timer >= self.AVOID_REACTIVE_MIN_TIME:
                    if self._on_mline(self.x, self.y):
                        d = math.hypot(self.x - self.end["x"], self.y - self.end["y"])
                        if d < self._mline_hit_dist:
                            self.avoid_state = "none"
                            self.avoid_timer = 0.0
                            self._avoid_cooldown = self.AVOID_COOLDOWN_S
                            self._avoid_cycles = 0
                            return self.max_speed, self.max_speed
                # Smart fallback: past hit point + front clear
                if (self.avoid_timer >= self.AVOID_REACTIVE_MIN_TIME
                        and self.x > self._mline_hit_x + 50
                        and f > 0 and f > self.AVOID_CLEAR_THRESHOLD_CM):
                    self.avoid_state = "none"
                    self.avoid_timer = 0.0
                    self._avoid_cooldown = self.AVOID_COOLDOWN_S
                    self._avoid_cycles = 0
                    return self.max_speed, self.max_speed
                # Timeout
                if self.avoid_timer >= self.AVOID_REACTIVE_TIMEOUT:
                    self.avoid_state = "none"
                    self.avoid_timer = 0.0
                    self._avoid_cycles = 0
                    return self.max_speed, self.max_speed
                # Stuck detection: dual-threshold
                self._drive_flip_time += dt
                traveled = math.hypot(
                    self.x - self._reactive_start_x,
                    self.y - self._reactive_start_y)
                if self.avoid_timer > 0.5 and traveled < 3.0:
                    self._avoid_cycles += 1
                    self.avoid_state = "reverse"
                    self.avoid_timer = 0.0
                    return -self.REVERSE_SPD, -self.REVERSE_SPD
                if self.avoid_timer > 2.0 and traveled < 5.0:
                    self._avoid_cycles += 1
                    self.avoid_state = "reverse"
                    self.avoid_timer = 0.0
                    return -self.REVERSE_SPD, -self.REVERSE_SPD
                # Oscillation detection
                if self._drive_flip_time > 1.0:
                    if self._drive_steer_flips >= 4:
                        self._avoid_cycles += 1
                        self.avoid_state = "reverse"
                        self.avoid_timer = 0.0
                        return -self.REVERSE_SPD, -self.REVERSE_SPD
                    self._drive_flip_time = 0.0
                    self._drive_steer_flips = 0
                # Steering: wall-following
                fwd = self.AVOID_REACTIVE_FWD_SPEED
                sl = self.sonar_left or 9999
                sr = self.sonar_right or 9999
                # Surrounded
                if f > 0 and f < 20 and min(sl, sr) < 15:
                    self._avoid_cycles += 1
                    self.avoid_state = "reverse"
                    self.avoid_timer = 0.0
                    return -self.REVERSE_SPD, -self.REVERSE_SPD
                T = self.AVOID_TARGET_DIST_CM
                if min(sl, sr) < T or (f > 0 and f < T * 2):
                    # Tight space: proportional wall-following
                    K = 0.02
                    if sl < sr:
                        err = sl - T
                        steer = -K * err
                        if f < T * 2:
                            steer += 0.15 * (1.0 - f / (T * 2))
                    else:
                        err = sr - T
                        steer = K * err
                        if f < T * 2:
                            steer -= 0.15 * (1.0 - f / (T * 2))
                    steer = clamp(steer, -self.TURN_SP, self.TURN_SP)
                elif f > 0 and f < T * 3:
                    # Moderate proximity
                    ratio = 1.0 - min(1.0, f / (T * 3))
                    steer = ratio * self.TURN_SP * 0.7
                    ts = -1 if sl > sr else 1
                    steer = steer * ts
                else:
                    self._drive_steer_flips = 0
                    self._drive_steer_sign = 0
                    return (fwd, fwd)
                # Track steer oscillation
                cur_sign = 1 if steer > 0 else -1 if steer < 0 else 0
                if cur_sign != 0 and cur_sign != self._drive_steer_sign:
                    if self._drive_steer_sign != 0:
                        self._drive_steer_flips += 1
                    self._drive_steer_sign = cur_sign
                return (fwd - steer, fwd + steer)

        if self._avoid_cooldown > 0:
            self._avoid_cooldown = max(0, self._avoid_cooldown - dt)
        if (self.avoid_state == "none" and self._avoid_cooldown <= 0
                and f > 0 and f < obs_dist):
            self.avoid_state = "reverse"
            self.avoid_timer = 0.0
            self._mline_hit_x = self.x
            self._mline_hit_y = self.y
            self._mline_hit_dist = math.hypot(self.x - self.end["x"], self.y - self.end["y"])
            self.pos_history = []
            return -self.REVERSE_SPD, -self.REVERSE_SPD

        if abs(goal_err) < self.HDG_TOL:
            return self.max_speed, self.max_speed

        turn_dir = math.copysign(1, goal_err)
        if abs(goal_err) > 0.3:
            return -self.TURN_SP * turn_dir, self.TURN_SP * turn_dir

        ratio = abs(goal_err) / 0.3
        inner = self.max_speed * (1 - ratio * 0.7)
        return (inner, self.max_speed) if goal_err > 0 else (self.max_speed, inner)

    # ------------------------------------------------------------------
    # Public API methods
    # ------------------------------------------------------------------

    def step(self, dt=0.05):
        """Advance simulation by one tick."""
        with self._lock:
            self._step_impl(dt)
            return self._build_state()

    def _step_impl(self, dt):
        self.sim_time += dt

        if self.sweep_requested:
            self._run_sim_sweep()

        if self.autopilot_on:
            n = self._nose_position()
            if self._sonar_override["front"] is not None:
                self.sonar_front = self._sonar_override["front"]
            else:
                val = self._raycast(n["x"], n["y"], self.theta)
                self.sonar_front = val if val is not None else 9999

            if self._sonar_override["left"] is not None:
                self.sonar_left = self._sonar_override["left"]
            else:
                val = self._raycast(n["x"], n["y"], self.theta - math.pi / 4)
                self.sonar_left = val if val is not None else 9999

            if self._sonar_override["right"] is not None:
                self.sonar_right = self._sonar_override["right"]
            else:
                val = self._raycast(n["x"], n["y"], self.theta + math.pi / 4)
                self.sonar_right = val if val is not None else 9999

            ls, rs = self._autopilot(dt)
            self.left_speed = ls
            self.right_speed = rs

        prev_x, prev_y = self.x, self.y
        self._step_physics(dt)
        if (self.autopilot_on and self.avoid_state == "none"
                and abs(self.x - prev_x) < 0.5 and abs(self.y - prev_y) < 0.5
                and self.left_speed > 0 and self.right_speed > 0):
            self._blocked_count += 1
            if self._blocked_count > 2:
                self.avoid_state = "reverse"
                self.avoid_timer = 0.0
                self._mline_hit_x = self.x
                self._mline_hit_y = self.y
                self._mline_hit_dist = math.hypot(self.x - self.end["x"], self.y - self.end["y"])
                self.pos_history = []
                self._blocked_count = 0
        else:
            self._blocked_count = 0

        self.trail.append({"x": self.x, "y": self.y})
        if len(self.trail) > 40:
            self.trail = self.trail[-40:]

        # Check arrival
        if self.autopilot_on:
            dist = math.hypot(self.x - self.end["x"], self.y - self.end["y"])
            if dist < self.ARRIVAL:
                self.arrived = True
                self.left_speed = 0.0
                self.right_speed = 0.0

        # Debug logging
        if self.autopilot_on:
            near = min(self.x, self.PW - self.x, self.y, self.PH - self.y)
            self.debug_buf.append({
                "t": f"{time.time() - self._start_time:.2f}",
                "x": f"{self.x:.1f}",   "y": f"{self.y:.1f}",
                "th": f"{math.degrees(self.theta):.1f}",
                "ls": f"{self.left_speed:.2f}",  "rs": f"{self.right_speed:.2f}",
                "f": f"{self.sonar_front if self.sonar_front else 0:.0f}",
                "l": f"{self.sonar_left if self.sonar_left else 0:.0f}",
                "r": f"{self.sonar_right if self.sonar_right else 0:.0f}",
                "near": f"{near:.0f}",
                "perimEscaping": self.perim_escaping,
                "avoidState": self.avoid_state,
                "perimCooldown": f"{self.perim_cooldown:.2f}",
            })
            if len(self.debug_buf) > 500:
                self.debug_buf = self.debug_buf[-500:]

        self.frame += 1

    def _build_state(self):
        near = min(self.x, self.PW - self.x, self.y, self.PH - self.y)
        bstate = "GO"
        if self.arrived:
            bstate = "ARRIVED"
        elif self.perim_escaping:
            bstate = "PERIM"
        elif self.avoid_state != "none":
            bstate = self.avoid_state.upper()
        elif not self.autopilot_on:
            bstate = "IDLE"
        wall_state = self.get_wall_state()
        return {
            "x_cm": round(self.x, 1),
            "y_cm": round(self.y, 1),
            "theta_rad": round(self.theta, 4),
            "left_speed": round(self.left_speed, 4),
            "right_speed": round(self.right_speed, 4),
            "sonar_front": round(self.sonar_front, 1) if self.sonar_front else None,
            "sonar_left": round(self.sonar_left, 1) if self.sonar_left else None,
            "sonar_right": round(self.sonar_right, 1) if self.sonar_right else None,
            "autopilot": self.autopilot_on,
            "brain_state": bstate,
            "avoid_state": self.avoid_state,
            "arrived": self.arrived,
            "inside": bool(self.x >= 1 and self.x <= self.PW - 1 and self.y >= 1 and self.y <= self.PH - 1),
            "edge_cm": round(near, 1),
            "frame": self.frame,
            "obstacles": list(self.obstacles),
            "start": dict(self.start),
            "end": dict(self.end),
            "trail": list(self.trail),
            "config": self._build_config(),
            "walls": wall_state,
        }

    def _build_config(self):
        return {
            "PW": self.PW, "PH": self.PH, "DUCK_R": self.DUCK_R,
            "MARGIN": self.MARGIN, "ARRIVAL": self.ARRIVAL,
            "OBST_TH": self.OBST_TH, "TURN_SP": self.TURN_SP,
            "REVERSE_SPD": self.REVERSE_SPD, "MAX_SPD": self.MAX_SPD,
            "WB": self.WB, "HDG_TOL": self.HDG_TOL,
            "AVOID_REVERSE_S": self.AVOID_REVERSE_S,
            "AVOID_COOLDOWN_S": self.AVOID_COOLDOWN_S,
            "AVOID_REACTIVE_FWD_SPEED": self.AVOID_REACTIVE_FWD_SPEED,
            "AVOID_TARGET_DIST_CM": self.AVOID_TARGET_DIST_CM,
            "AVOID_REACTIVE_TIMEOUT": self.AVOID_REACTIVE_TIMEOUT,
            "AVOID_REACTIVE_MIN_TIME": self.AVOID_REACTIVE_MIN_TIME,
            "AVOID_CLEAR_THRESHOLD_CM": self.AVOID_CLEAR_THRESHOLD_CM,
            "STUCK_DIST_CM": self.STUCK_DIST_CM,
            "STUCK_WINDOW_S": self.STUCK_WINDOW_S,
            "max_speed": self.max_speed,
        }

    # --- state modifiers ---

    def get_state(self):
        with self._lock:
            return self._build_state()

    def get_debug(self, limit=50):
        with self._lock:
            return {"frames": self.debug_buf[-limit:]}

    def get_config(self):
        with self._lock:
            return self._build_config()

    def get_obstacles(self):
        with self._lock:
            return list(self.obstacles)

    def set_pose(self, x, y, theta=None):
        with self._lock:
            self.x = float(x)
            self.y = float(y)
            if theta is not None:
                self.theta = float(theta)
            else:
                self.theta = 0.0
            self.autopilot_on = False
            self.arrived = False
            self.avoid_state = "none"
            self.avoid_timer = 0.0
            self._avoid_cooldown = 0.0
            self.pos_history = []
            self.perim_escaping = False
            self.perim_cooldown = 0.0
            self.left_speed = 0.0
            self.right_speed = 0.0
            return self._build_state()

    def reset(self):
        return self.set_pose(self.start["x"], self.start["y"], 0.0)

    def set_speeds(self, left, right):
        with self._lock:
            self.left_speed = clamp(float(left), -1, 1)
            self.right_speed = clamp(float(right), -1, 1)
            self.autopilot_on = False
            self.arrived = False
            return self._build_state()

    def set_autopilot(self, on):
        with self._lock:
            self.autopilot_on = bool(on)
            self.arrived = False
            if not on:
                self.left_speed = 0.0
                self.right_speed = 0.0
            return self._build_state()

    def set_sonar_override(self, front=None, left=None, right=None):
        with self._lock:
            for key, val in (("front", front), ("left", left), ("right", right)):
                self._sonar_override[key] = float(val) if val is not None else None
            return self._build_state()

    def add_obstacle(self, x, y, r=None):
        with self._lock:
            oid = self._next_obs_id
            self._next_obs_id += 1
            if r is None:
                r = 10.0
            self.obstacles.append({"id": oid, "x": float(x), "y": float(y), "r": float(r)})
            return oid

    def remove_obstacle(self, oid):
        with self._lock:
            for i, o in enumerate(self.obstacles):
                if o["id"] == oid:
                    self.obstacles.pop(i)
                    return True
            return False

    def clear_obstacles(self):
        with self._lock:
            self.obstacles.clear()

    def set_walls_visible(self, visible):
        with self._lock:
            self.wall_map_visible = bool(visible)
            return {"ok": True}

    def get_wall_state(self):
        if not self.wall_map_visible or self.wall_map is None:
            return None
        return self.wall_map.to_dict()

    def request_sweep(self):
        with self._lock:
            self.sweep_requested = True
            self.wall_map_visible = True
            return {"ok": True}

    def _run_sim_sweep(self):
        origin_x = self.x
        origin_y = self.y

        def sonar_at_angle(phi_rad):
            rng = self._raycast(origin_x, origin_y, phi_rad)
            if rng is not None and rng > 20:
                return rng
            return None

        readings, _meta = simulate_sweep(
            sonar_at_angle,
            duck_x=self.x,
            duck_y=self.y,
        )

        wm = WallMap()
        wm.init_from_minima(readings, duck_x=self.x, duck_y=self.y)
        self.wall_map = wm
        self.sweep_readings = readings
        self.sweep_requested = False
        return wm

    def hide_walls(self):
        with self._lock:
            self.wall_map_visible = False
            self.wall_map = None
            self.sweep_requested = False
            return {"ok": True}

    def set_goal(self, start=None, end=None):
        with self._lock:
            if start is not None:
                self.start["x"] = float(start["x"])
                self.start["y"] = float(start["y"])
                self.x = self.start["x"]
                self.y = self.start["y"]
                self.theta = 0.0
                self.arrived = False
                self.avoid_state = "none"
                self.avoid_timer = 0.0
                self._avoid_cooldown = 0.0
                self.pos_history = []
        if end is not None:
            self.end["x"] = float(end["x"])
            self.end["y"] = float(end["y"])
            self.arrived = False
            self.avoid_state = "none"
            self.avoid_timer = 0.0
            self._avoid_cooldown = 0.0
            self.perim_escaping = False
            self.perim_cooldown = 0.0
            self.pos_history = []
            return self._build_state()

    def update_config(self, data):
        with self._lock:
            for key, val in data.items():
                if hasattr(self, key):
                    setattr(self, key, float(val) if isinstance(val, (int, float)) else val)
            return self._build_config()

    def load_scenario(self, data):
        with self._lock:
            if data.get("config"):
                for k, v in data["config"].items():
                    if hasattr(self, k):
                        setattr(self, k, float(v) if isinstance(v, (int, float)) else v)
            obs = data.get("obstacles") or []
            self.obstacles.clear()
            for o in obs:
                oid = self._next_obs_id
                self._next_obs_id += 1
                self.obstacles.append({"id": oid, "x": float(o["x"]), "y": float(o["y"]), "r": float(o.get("r", 10))})
            st = data.get("start")
            en = data.get("end")
            if st:
                self.start["x"] = float(st["x"])
                self.start["y"] = float(st["y"])
            if en:
                self.end["x"] = float(en["x"])
                self.end["y"] = float(en["y"])
            self.x = self.start["x"]
            self.y = self.start["y"]
            self.theta = 0.0
            self.arrived = False
            self.autopilot_on = False
            self.avoid_state = "none"
            self.avoid_timer = 0.0
            self._avoid_cooldown = 0.0
            self.pos_history = []
            self.perim_escaping = False
            self.escape_heading = 0.0
            self.perim_cooldown = 0.0
            self.left_speed = 0.0
            self.right_speed = 0.0
            self._sonar_override = {"front": None, "left": None, "right": None}
            self.trail.clear()
            self.debug_buf.clear()
            self.frame = 0
            return self._build_state()
