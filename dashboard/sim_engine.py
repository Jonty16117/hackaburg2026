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

from navigation.grid import Grid
from navigation.jps import jps_search
from navigation.path_follower import PathFollower
from navigation.utils import clamp, normalize_angle, heading_error


class SimEngine:
    """Port of the browser duck simulation — physics + autopilot."""

    def __init__(self):
        self._lock = threading.Lock()

        # --- geometry ---
        self.PW = 1000
        self.PH = 200
        self.DUCK_R = 15
        self.MARGIN = 30
        self.ARRIVAL = 20

        # --- autopilot config (mutable) ---
        self.OBST_TH = 25
        self.TURN_SP = 0.5
        self.REVERSE_SPD = 0.5
        self.MAX_SPD = 100
        self.WB = 30
        self.HDG_TOL = 0.02
        self.AVOID_REVERSE_S = 0.5
        self.AVOID_COOLDOWN_S = 0.3
        self.STUCK_DIST_CM = 8
        self.STUCK_WINDOW_S = 1.5
        self.max_speed = 0.6

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

        # --- autopilot FSM ---
        self.autopilot_on = False
        self.arrived = False
        self.avoid_state = "none"       # none | reverse | scan | face_best | cooldown
        self.avoid_timer = 0.0
        self.pos_history = []
        self.scan_swept = 0.0
        self.scan_best_d = 0.0
        self.scan_best_th = 0.0
        self.scan_debounce = 0.0
        self.scan_dir = 1
        self.perim_escaping = False
        self.escape_heading = 0.0
        self.perim_cooldown = 0.0

        # --- JPS path following ---
        self._grid = None
        self._path = []
        self._path_follower = None
        self._replan_count = 0

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

    # --- grid / path helpers ---

    def _replan_path(self):
        margin = self.DUCK_R + 3
        self._grid = Grid(self.PW, self.PH, 2, list(self.obstacles), margin)
        sp = (self.x, self.y)
        ep = (self.end["x"], self.end["y"])
        eg = self._grid.world_to_grid(*ep)
        if self._grid.is_blocked(eg[0], eg[1]):
            for r in range(1, 30):
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        nx, ny = eg[0] + dx, eg[1] + dy
                        if self._grid.in_bounds(nx, ny) and not self._grid.is_blocked(nx, ny):
                            ep = self._grid.grid_to_world(nx, ny)
                            break
                    else:
                        continue
                    break
                else:
                    continue
                break
        path = jps_search(self._grid, sp, ep)
        if path:
            self._path = path
            self._path_follower = PathFollower(path, self.max_speed, self.HDG_TOL,
                                               arrival_dist=self.ARRIVAL)
            self._replan_count += 1
        else:
            self._path = []
            self._path_follower = None

    # --- autopilot FSM (port of JS autoPilot) + JPS integration ---
    def _autopilot(self, dt):
        dist_end = math.hypot(self.x - self.end["x"], self.y - self.end["y"])
        if dist_end < self.ARRIVAL:
            self.arrived = True
            self._path = []
            self._path_follower = None
            return 0.0, 0.0
        if self.arrived:
            self.arrived = False

        f = self.sonar_front if self.sonar_front is not None else 9999
        obs_dist = self.OBST_TH + self.DUCK_R

        # --- JPS path following ---
        if not self._path and self.avoid_state == "none" and not self.perim_escaping:
            self._replan_path()

        if self._path and self._path_follower and not self.perim_escaping:
            if self.avoid_state == "none":
                ls, rs = self._path_follower.compute_speeds(self.x, self.y, self.theta)
                if self._path_follower.arrived:
                    self.arrived = True
                    self._path = []
                    self._path_follower = None
                    return 0.0, 0.0
                return ls, rs
            elif self.avoid_state == "reverse":
                self.avoid_timer += dt
                if self.avoid_timer >= self.AVOID_REVERSE_S:
                    self.avoid_state = "none"
                    self.avoid_timer = 0.0
                    self._replan_path()
                return -self.REVERSE_SPD, -self.REVERSE_SPD

        goal_th = math.atan2(self.end["y"] - self.y, self.end["x"] - self.x)
        goal_err = heading_error(goal_th, self.theta)

        # Stuck detection
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
                self.pos_history = []
                return -self.REVERSE_SPD, -self.REVERSE_SPD

        # Perimeter escape detection
        near = min(self.x, self.PW - self.x, self.y, self.PH - self.y)
        if near < self.MARGIN and not self.perim_escaping and self.perim_cooldown <= 0:
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
            self.avoid_state = "none"
            self.avoid_timer = 0.0
            self.pos_history = []

        if near > self.MARGIN * 2 and self.perim_escaping:
            self.perim_escaping = False
            self.perim_cooldown = 0.5

        if self.perim_cooldown > 0:
            self.perim_cooldown = max(0, self.perim_cooldown - dt)

        if self.perim_escaping:
            perr = heading_error(self.escape_heading, self.theta)
            if abs(perr) < 0.08:
                return self.max_speed, self.max_speed
            return -math.copysign(1, perr), math.copysign(1, perr)

        if self.perim_cooldown > 0:
            cperr = heading_error(self.escape_heading, self.theta)
            if abs(cperr) < 0.08:
                return self.max_speed, self.max_speed
            return -math.copysign(1, cperr), math.copysign(1, cperr)

        # Avoid FSM (fallback when no JPS path)
        if self.avoid_state != "none":
            self.avoid_timer += dt
            if self.avoid_state == "reverse":
                if self.avoid_timer >= self.AVOID_REVERSE_S:
                    self.avoid_state = "scan"
                    self.avoid_timer = 0.0
                    self.scan_swept = 0.0
                    self.scan_best_d = 0.0
                    self.scan_best_th = self.theta
                    self.scan_debounce = 0.3
                    self.scan_dir = -self.scan_dir
                else:
                    return -self.REVERSE_SPD, -self.REVERSE_SPD
            if self.avoid_state == "scan":
                self.scan_swept += 2 * self.TURN_SP * self.MAX_SPD / self.WB * dt
                if self.scan_debounce > 0:
                    self.scan_debounce -= dt
                elif f > self.scan_best_d:
                    self.scan_best_d = f
                    self.scan_best_th = self.theta
                if self.scan_swept >= math.pi / 2:
                    self.avoid_state = "face_best"
                    self.avoid_timer = 0.0
                return (-self.TURN_SP * self.scan_dir,
                        self.TURN_SP * self.scan_dir)
            if self.avoid_state == "face_best":
                berr = heading_error(self.scan_best_th, self.theta)
                if abs(berr) < self.HDG_TOL:
                    self.avoid_state = "cooldown"
                    self.avoid_timer = 0.0
                else:
                    turn_dir = math.copysign(1, berr)
                    rot = min(1.0, abs(berr) / 0.5) * self.TURN_SP
                    return -rot * turn_dir, rot * turn_dir
            if self.avoid_state == "cooldown":
                if self.avoid_timer >= self.AVOID_COOLDOWN_S:
                    self.avoid_state = "none"
                    self.avoid_timer = 0.0
                else:
                    return self.max_speed * 0.5, self.max_speed * 0.5

        if self.avoid_state == "none" and f > 0 and f < obs_dist:
            self.avoid_state = "reverse"
            self.avoid_timer = 0.0
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
        # Bump detection: if duck tried to move forward but was blocked by obstacle
        if (self.autopilot_on and self.avoid_state == "none"
                and self.x == prev_x and self.y == prev_y
                and self.left_speed > 0 and self.right_speed > 0):
            self._blocked_count += 1
            if self._blocked_count > 3:  # 4+ consecutive blocks = stuck
                self.avoid_state = "reverse"
                self.avoid_timer = 0.0
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
            "jps_path": [(round(x, 1), round(y, 1)) for x, y in self._path],
            "jps_path_len": len(self._path),
            "replan_count": self._replan_count,
            "config": self._build_config(),
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

    def _clear_path(self):
        self._path = []
        self._path_follower = None
        self._grid = None

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
            self.pos_history = []
            self.scan_swept = 0.0
            self.scan_best_d = 0.0
            self.scan_best_th = 0.0
            self.scan_debounce = 0.0
            self.perim_escaping = False
            self.perim_cooldown = 0.0
            self.left_speed = 0.0
            self.right_speed = 0.0
            self._clear_path()
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
            self._clear_path()
            return oid

    def remove_obstacle(self, oid):
        with self._lock:
            for i, o in enumerate(self.obstacles):
                if o["id"] == oid:
                    self.obstacles.pop(i)
                    self._clear_path()
                    return True
            return False

    def clear_obstacles(self):
        with self._lock:
            self.obstacles.clear()
            self._clear_path()

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
                self.pos_history = []
                self._clear_path()
        if end is not None:
            self.end["x"] = float(end["x"])
            self.end["y"] = float(end["y"])
            self.arrived = False
            self.avoid_state = "none"
            self.avoid_timer = 0.0
            self.perim_escaping = False
            self.perim_cooldown = 0.0
            self.pos_history = []
            self._clear_path()
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
            self.pos_history = []
            self._clear_path()
            self.scan_swept = 0.0
            self.scan_best_d = 0.0
            self.scan_best_th = 0.0
            self.scan_debounce = 0.0
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
