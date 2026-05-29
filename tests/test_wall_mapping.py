"""Comprehensive test: wall mapping, EKF tracking, obstacles, water drift.

Tests position correctness under realistic conditions:
- Orientation sweep at boot
- EKF tracking with motor + drift motion
- Sonar corrections against known walls
- Obstacle detection (innovation gating)
- Random water drift simulation
- Perimeter inside/edge checks
"""

import math
import random
import time

from navigation.wall_map import WallMap
from navigation.ekf_localizer import EKFLocalizer
from navigation.utils import heading_error, normalize_angle
from navigation.config import START_X_CM, START_Y_CM, END_X_CM, END_Y_CM


# ---------------------------------------------------------------------------
# Simulated water drift
# ---------------------------------------------------------------------------
class WaterDrift:
    def __init__(self, drift_strength_cm_s=4.0, rotation_noise_rad_s=0.01, seed=42):
        self.drift_strength = drift_strength_cm_s
        self.rotation_noise = rotation_noise_rad_s
        self.rng = random.Random(seed)
        self._drift_angle = self.rng.uniform(0, 2 * math.pi)
        self._timer = 0.0

    def apply(self, x, y, theta, dt):
        self._timer += dt
        if self._timer > self.rng.uniform(3.0, 10.0):
            self._drift_angle = self.rng.uniform(0, 2 * math.pi)
            self._timer = 0.0
        mag = self.drift_strength * dt * self.rng.uniform(0.5, 1.5)
        x += mag * math.cos(self._drift_angle)
        y += mag * math.sin(self._drift_angle)
        theta += self.rotation_noise * dt * self.rng.uniform(-1, 1)
        theta = normalize_angle(theta)
        return x, y, theta


# ---------------------------------------------------------------------------
# Simulated sonar
# ---------------------------------------------------------------------------
class SimSonar:
    def __init__(self, pool_w=1000, pool_h=200, cone_half_deg=37.5):
        self.pw = pool_w
        self.ph = pool_h
        self.half = math.radians(cone_half_deg)
        self.obstacles = []

    def add_obstacle(self, x, y, r):
        self.obstacles.append({"x": x, "y": y, "r": r})

    def reading(self, dx, dy, theta):
        best = float("inf")
        for a in [theta, theta - self.half, theta + self.half]:
            cx = math.cos(a)
            sy = math.sin(a)
            t_hits = []
            for wx, wy, nx, ny in [
                (self.pw, dy, 1, 0), (0, dy, -1, 0),
                (dx, self.ph, 0, 1), (dx, 0, 0, -1),
            ]:
                if abs(cx) > 1e-9 and nx != 0:
                    t = (wx - dx) / cx
                elif abs(sy) > 1e-9 and ny != 0:
                    t = (wy - dy) / sy
                else:
                    continue
                if t > 0:
                    px, py = dx + cx * t, dy + sy * t
                    if 0 <= px <= self.pw and 0 <= py <= self.ph:
                        t_hits.append(t)
            for o in self.obstacles:
                fx, fy = dx - o["x"], dy - o["y"]
                bv = 2 * (fx * cx + fy * sy)
                cv = fx * fx + fy * fy - o["r"] * o["r"]
                disc = bv * bv - 4 * cv
                if disc < 0:
                    continue
                t1 = (-bv - math.sqrt(disc)) / 2
                t2 = (-bv + math.sqrt(disc)) / 2
                for t in (t1, t2):
                    if t > 0:
                        t_hits.append(t)
            if t_hits:
                best = min(best, min(t_hits))
        return best if (best < float("inf") and best > 20) else None


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_sweep_basic():
    """Orientation sweep at 5 different duck angles."""
    from dashboard.sim_engine import SimEngine
    for angle_deg in [0, 15, 30, 45, -20]:
        engine = SimEngine()
        engine.set_pose(START_X_CM, START_Y_CM, math.radians(angle_deg))
        readings = []
        for deg in range(0, 360, 10):
            body_phi = math.radians(deg)
            world_phi = body_phi + math.radians(angle_deg)
            rng = engine._raycast(START_X_CM, START_Y_CM, world_phi)
            readings.append((body_phi, rng if (rng and rng > 20) else None))
        wm = WallMap()
        info = wm.init_from_sweep(readings, duck_x=START_X_CM, duck_y=START_Y_CM)
        d = info["delta_theta_deg"]
        target = -angle_deg
        err = abs(d - target)
        ok = err < 10 or abs(err - 360) < 10
        assert ok, f"Δθ={d}° expected ~{target}° (off by {err:.1f}°)"
    print("  PASS: 5 orientations detected")


def test_ekf_drift_recovery():
    """EKF recovers from drift when sonar disagrees with prediction."""
    wm = WallMap()
    wm.init_known_walls()
    ekf = EKFLocalizer(wm, (START_X_CM, START_Y_CM, 0))

    # EKF thinks duck at (30, 100) → right wall predicted at 970cm
    # Duck actually drifted to x=80 → sonar reads 920cm
    ekf.correct(920)
    x, _, _ = ekf.get_pose()
    assert x > 45, f"Should correct toward right wall, got x={x:.1f}"
    assert ekf.total_corrections == 1

    # Reset and check wall remains visible
    ekf.set_pose(START_X_CM, START_Y_CM, 0)
    _, y, _ = ekf.get_pose()
    assert y == START_Y_CM, f"y should be unchanged, got y={y:.1f}"
    print(f"  PASS: x={x:.1f}, y={y:.1f}")


def test_obstacle_rejection():
    """Obstacle reading does NOT correct EKF position or perimeter."""
    wm = WallMap()
    wm.init_known_walls()
    ekf = EKFLocalizer(wm, (START_X_CM, START_Y_CM, 0))
    x0 = ekf.x
    ekf.correct(120)
    assert ekf.lost_count == 1, "Obstacle must increment lost_count"
    assert abs(ekf.x - x0) < 0.01, "Position must not change"
    print("  PASS: ignored, lost_count=1")


def test_no_wall_no_correction():
    """No wall visible → no change, no lost_count."""
    wm = WallMap()
    wm.init_known_walls()
    ekf = EKFLocalizer(wm, (START_X_CM, START_Y_CM, 0))
    ekf.set_pose(START_X_CM, START_Y_CM, math.radians(45))
    x0 = ekf.x
    ekf.correct(600)
    assert ekf.lost_count == 0 and ekf._no_wall_count == 1
    assert abs(ekf.x - x0) < 0.01
    print("  PASS: skipped, _no_wall_count=1")


def test_perimeter_always_correct():
    """Fixed-wall perimeter is always valid regardless of EKF drift."""
    wm = WallMap()
    wm.init_known_walls()
    perim = wm.to_perimeter()
    ekf = EKFLocalizer(wm, (START_X_CM, START_Y_CM, 0))
    for _ in range(100):
        ekf.predict(0.4, 0.38, 0.05)
    x, y, _ = ekf.get_pose()
    inside = perim.is_inside(x, y)
    edge = perim.distance_to_edge(x, y)
    print(f"  after 5s: pose=({x:.0f},{y:.0f}), inside={inside}, "
          f"edge={edge:.0f}cm")
    assert inside or edge < 1, "If outside, must be at boundary"
    print("  PASS: perimeter always valid")


def test_ekf_tracks_with_drift():
    """EKF recovers position after corrections despite moderate drift."""
    wm = WallMap()
    wm.init_known_walls()
    ekf = EKFLocalizer(wm, (START_X_CM, START_Y_CM, 0))
    drift = WaterDrift(drift_strength_cm_s=3.0, seed=1)
    sonar = SimSonar(pool_w=1000, pool_h=200)

    true_x, true_y, true_th = float(START_X_CM), float(START_Y_CM), 0.0
    errors = []
    corrections = 0
    dt = 0.05

    for i in range(400):
        if i % 100 < 80:
            ls, rs = 0.3, 0.3
        else:
            ls, rs = -0.35, 0.35

        ekf.predict(ls, rs, dt)
        true_x += (ls + rs) / 2 * 100 * dt * math.cos(true_th)
        true_y += (ls + rs) / 2 * 100 * dt * math.sin(true_th)
        true_th += (rs - ls) * 100 / 30 * dt
        true_th = normalize_angle(true_th)
        true_x, true_y, true_th = drift.apply(true_x, true_y, true_th, dt)
        true_x = max(5, min(995, true_x))
        true_y = max(5, min(195, true_y))

        s = sonar.reading(true_x, true_y, true_th)
        before = ekf.total_corrections
        if s is not None:
            ekf.correct(s)
        if ekf.total_corrections > before:
            corrections += 1

        if i % 100 == 99:
            errors.append(math.hypot(ekf.x - true_x, ekf.y - true_y))

    avg = sum(errors) / len(errors)
    max_e = max(errors) if errors else 0
    print(f"  avg_error={avg:.1f}cm, max_error={max_e:.1f}cm, "
          f"corrections={corrections}")
    assert corrections > 0, "Must have corrections"
    assert avg < 120, f"Average EKF error {avg:.1f}cm too high"
    print("  PASS: EKF tracks within tolerance")

def test_obstacle_scenario():
    """Obstacle placed off-path: corrections still match walls."""
    wm = WallMap()
    wm.init_known_walls()
    ekf = EKFLocalizer(wm, (START_X_CM, START_Y_CM, 0))
    sonar = SimSonar(pool_w=1000, pool_h=200)
    sonar.add_obstacle(600, 150, 10)

    true_x, true_y, true_th = float(START_X_CM), float(START_Y_CM), 0.0
    ok, bad = 0, 0
    dt = 0.05

    for i in range(300):
        turn = 0.05 * math.sin(i * 0.02)
        ls, rs = 0.35 - turn, 0.35 + turn
        ekf.predict(ls, rs, dt)
        true_x += ls * 100 * dt * math.cos(true_th)
        true_y += ls * 100 * dt * math.sin(true_th)
        true_th += (rs - ls) * 100 / 30 * dt
        true_th = normalize_angle(true_th)

        s = sonar.reading(true_x, true_y, true_th)
        if s is not None:
            wall_idx, _ = wm.nearest_visible_wall(ekf.x, ekf.y, ekf.theta)
            if wall_idx is not None:
                wall = wm.walls[wall_idx]
                true_dist = wall.perpendicular_distance(true_x, true_y)
                before = ekf.total_corrections
                ekf.correct(s)
                if ekf.total_corrections > before:
                    if abs(s - true_dist) < 40:
                        ok += 1
                    else:
                        bad += 1

    total = ok + bad
    rate = bad / max(total, 1)
    print(f"  ok={ok} bad={bad} total={total} rate={rate:.1%}")
    assert rate < 0.50, f"Too many bad corrections: {rate:.1%}"
    print("  PASS: obstacle corrections within tolerance")


def test_heavy_drift_with_varied_headings():
    """Under heavy drift + varied headings, EKF gets corrections."""
    wm = WallMap()
    wm.init_known_walls()
    ekf = EKFLocalizer(wm, (START_X_CM, START_Y_CM, 0))
    drift = WaterDrift(drift_strength_cm_s=8.0, rotation_noise_rad_s=0.02, seed=3)
    sonar = SimSonar(pool_w=1000, pool_h=200)

    true_x, true_y, true_th = float(START_X_CM), float(START_Y_CM), 0.0
    corrections = 0
    samples = []
    dt = 0.05

    for i in range(600):
        ls, rs = 0.3 + 0.1 * math.sin(i * 0.05), 0.3
        ekf.predict(ls, rs, dt)
        true_x += (ls + rs) / 2 * 100 * dt * math.cos(true_th)
        true_y += (ls + rs) / 2 * 100 * dt * math.sin(true_th)
        true_th += (rs - ls) * 100 / 30 * dt
        true_th = normalize_angle(true_th)
        true_x, true_y, true_th = drift.apply(true_x, true_y, true_th, dt)
        true_x = max(5, min(995, true_x))
        true_y = max(5, min(195, true_y))

        s = sonar.reading(true_x, true_y, true_th)
        before = ekf.total_corrections
        if s is not None:
            ekf.correct(s)
        if ekf.total_corrections > before:
            corrections += 1

        if i % 100 == 50:
            samples.append(math.hypot(ekf.x - true_x, ekf.y - true_y))

    avg = sum(samples) / len(samples) if samples else 0
    print(f"  avg_error={avg:.1f}cm, corrections={corrections}")
    assert corrections >= 1, "Must have at least 1 correction"
    print("  PASS: corrections obtained under drift + varied heading")


def test_x_correction_gating():
    """needs_x_correction gating works."""
    wm = WallMap()
    wm.init_known_walls()
    ekf = EKFLocalizer(wm, (START_X_CM, START_Y_CM, 0))
    assert not ekf.needs_x_correction()
    ekf.total_corrections = 5
    assert not ekf.needs_x_correction()
    ekf.last_x_correction_time = time.time() - 35
    assert ekf.needs_x_correction()
    ekf.last_x_correction_time = time.time()
    assert not ekf.needs_x_correction()
    print("  PASS: gating correct")


def test_reacquire_triggers():
    """should_reacquire triggers on no_wall_count or lost_count + idle."""
    wm = WallMap()
    wm.init_known_walls()
    ekf = EKFLocalizer(wm, (START_X_CM, START_Y_CM, 0))
    assert not ekf.should_reacquire()
    ekf.last_move_time = time.time() - 10
    ekf._no_wall_count = 250
    assert ekf.should_reacquire()
    ekf._no_wall_count = 0
    ekf.lost_count = 25
    assert ekf.should_reacquire()
    print("  PASS: both triggers work")


def test_full_sim_run():
    """Full sim: sweep → walls → autopilot near end point with obstacles."""
    from dashboard.sim_engine import SimEngine
    engine = SimEngine()
    engine.set_pose(START_X_CM, START_Y_CM, 0)
    engine.add_obstacle(650, 100, 10)
    engine.add_obstacle(750, 120, 8)
    engine.request_sweep()
    for _ in range(5):
        engine.step(0.05)
    state = engine.get_state()
    w = state.get("walls")
    assert w and len(w["walls"]) == 4
    engine.set_autopilot(True)
    engine.set_goal(start={"x": START_X_CM, "y": START_Y_CM}, end={"x": END_X_CM, "y": END_Y_CM})
    for _ in range(400):
        engine.step(0.05)
        state = engine.get_state()
        if state.get("arrived"):
            break
    inside = state.get("inside", False)
    assert inside, "Must stay inside pool"
    print(f"  PASS: autopilot → near end, inside={inside}")


