"""DuckBot autonomous navigation controller.

Runs boot-time wall orientation sweep, tracks position via EKF,
and sends motor commands at ~20 Hz.

Usage:
    python -m navigation.controller
"""

import math
import time

from motors.i2c_drive import I2C_ADDR, I2C_BUS
from navigation.config import (
    SONAR_L_TRIG, SONAR_L_ECHO,
    SONAR_F_TRIG, SONAR_F_ECHO,
    SONAR_R_TRIG, SONAR_R_ECHO,
    PERIMETER_CM, START_X_CM, START_Y_CM, START_HEADING_RAD,
    END_X_CM, END_Y_CM, BRAIN_CFG, MAPPER_TOGGLE,
    MAPPER_SWEEP_SPEED, MAPPER_SWEEP_DEG_STEP, MAPPER_SONAR_SAMPLES,
    MAPPER_BLIND_ZONE_CM,
)
from navigation.odometry import Odometry
from navigation.perimeter import Perimeter
from navigation.brain import Brain, State, _AvoidPhase
from navigation.wall_map import WallMap
from navigation.ekf_localizer import EKFLocalizer
from navigation.utils import heading_error


AVOID_PHASE_LABELS = {
    _AvoidPhase.REVERSE:         "AVOID:REV",
    _AvoidPhase.TURN_AND_SENSE:  "AVOID:SCAN",
    _AvoidPhase.FACE_OPENING:    "AVOID:FACE",
    _AvoidPhase.REACTIVE_DRIVE:  "AVOID:DRIVE",
}


def _state_label(brain):
    if brain.state == State.AVOID:
        return AVOID_PHASE_LABELS.get(brain._avoid_phase, "AVOID")
    return brain.state.name


def run_navigation(on_cycle=None, paused=None, stop_event=None):
    import RPi.GPIO as GPIO
    from motors.i2c_drive import I2CDrive
    from sensors.sonar import Sonar
    from navigation.sonar_sweep import SonarSweep

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    drive = I2CDrive()
    sonar_l = Sonar(SONAR_L_TRIG, SONAR_L_ECHO)
    sonar_f = Sonar(SONAR_F_TRIG, SONAR_F_ECHO)
    sonar_r = Sonar(SONAR_R_TRIG, SONAR_R_ECHO)
    sonars = [sonar_l, sonar_f, sonar_r]
    odom = Odometry(START_X_CM, START_Y_CM, START_HEADING_RAD,
                    BRAIN_CFG["WHEEL_BASE_CM"], BRAIN_CFG["MAX_SPEED_CM_S"])

    wall_map = WallMap()
    ekf = None
    sweep_info = None

    if MAPPER_TOGGLE:
        print("=" * 60)
        print(" Wall orientation sweep running... (~18s)")
        print("=" * 60)
        try:
            sweeper = SonarSweep(
                drive, sonar_f,
                sweep_speed=MAPPER_SWEEP_SPEED,
                step_deg=MAPPER_SWEEP_DEG_STEP,
                sonar_samples=MAPPER_SONAR_SAMPLES,
                blind_zone_cm=MAPPER_BLIND_ZONE_CM,
            )
            sweep_result = sweeper.run(odom)
            sweep_info = wall_map.init_from_sweep(
                sweep_result["readings"],
                duck_x=START_X_CM,
                duck_y=START_Y_CM,
            )

            print(f"\n Sweep complete: "
                  f"Δθ={sweep_info['delta_theta_deg']}° "
                  f"({sweep_info['walls_seen']}/4 walls seen)")
            if sweep_info["warning"]:
                print(f" {sweep_info['prefix']}: {sweep_info['warning']}")
        except Exception as e:
            print(f" Sweep failed: {e}")
            print(" Using Δθ=0 (hardcoded orientation)")
            wall_map.init_known_walls(delta_theta=0.0)
    else:
        wall_map.init_known_walls(delta_theta=0.0)

    perim = Perimeter(PERIMETER_CM)
    brain = Brain(perim, BRAIN_CFG, goal_x=END_X_CM, goal_y=END_Y_CM)

    ekf = EKFLocalizer(
        wall_map,
        (START_X_CM, START_Y_CM, START_HEADING_RAD),
        wheel_base_cm=BRAIN_CFG["WHEEL_BASE_CM"],
        max_speed_cm_s=BRAIN_CFG["MAX_SPEED_CM_S"],
    )

    print("=" * 72)
    print("DuckBot — Autonomous Navigation Controller")
    if sweep_info:
        print(f"  Orientation : Δθ={sweep_info['delta_theta_deg']}° "
              f"({sweep_info['walls_seen']}/4 walls confirmed)")
    print(f"  Start pose  : ({START_X_CM}, {START_Y_CM})  "
          f"@{math.degrees(START_HEADING_RAD):.0f}°")
    print(f"  End          : ({END_X_CM}, {END_Y_CM})")
    print(f"  Sonar (L): TRIG=GPIO{SONAR_L_TRIG}  ECHO=GPIO{SONAR_L_ECHO}")
    print(f"  Sonar (F): TRIG=GPIO{SONAR_F_TRIG}  ECHO=GPIO{SONAR_F_ECHO}")
    print(f"  Sonar (R): TRIG=GPIO{SONAR_R_TRIG}  ECHO=GPIO{SONAR_R_ECHO}")
    print(f"  Motors      : I2C addr=0x{I2C_ADDR:02X}  bus={I2C_BUS}")
    print(f"  Loop rate   : {BRAIN_CFG['LOOP_HZ']} Hz")
    print("  Press Ctrl+C to stop.")
    print("=" * 72)

    if paused is not None:
        print("  Status      : PAUSED — click Start in dashboard to begin")
        print("=" * 72)

    ll, lr = 0.0, 0.0
    lt = time.time()
    last_log = [lt]

    _x_correction_active = False
    _x_correction_start = 0.0
    _x_correction_timeout = 6.0

    def _safe_read(s):
        try:
            return s.distance_cm()
        except Exception:
            return None

    try:
        while True:
            if stop_event and stop_event.is_set():
                print("Nav stop requested — exiting")
                break
            if paused and paused.is_set():
                drive.stop()
                dl = _safe_read(sonar_l)
                df = _safe_read(sonar_f)
                dr = _safe_read(sonar_r)
                x, y, theta = ekf.get_pose()
                if on_cycle:
                    data = dict(
                        d=None, dl=dl, df=df, dr=dr,
                        x=x, y=y, theta=theta, ls=0.0, rs=0.0,
                        brain=brain, perim=perim, walls=wall_map.to_dict(),
                    )
                    on_cycle(data)
                time.sleep(0.1)
                continue

            now = time.time()
            dt = max(now - lt, 0.001)
            lt = now

            odom.update(ll, lr, dt)
            ekf.predict(ll, lr, dt)

            dl = _safe_read(sonar_l)
            time.sleep(0.02)
            df = _safe_read(sonar_f)
            time.sleep(0.02)
            dr = _safe_read(sonar_r)
            valid = [v for v in (dl, df, dr) if v is not None]
            d = min(valid) if valid else None
            if d is not None:
                ekf.correct(d)

            if ekf.should_reacquire():
                _x_correction_active = False
                print("\n EKF lost while idle — re-running sweep...")
                try:
                    sweeper = SonarSweep(
                        drive, sonar_f,
                        sweep_speed=MAPPER_SWEEP_SPEED,
                        step_deg=MAPPER_SWEEP_DEG_STEP,
                        sonar_samples=MAPPER_SONAR_SAMPLES,
                        blind_zone_cm=MAPPER_BLIND_ZONE_CM,
                    )
                    result = sweeper.run(odom)
                    sweep_info = wall_map.init_from_sweep(
                        result["readings"],
                        duck_x=START_X_CM,
                        duck_y=START_Y_CM,
                    )
                    ekf.set_pose(START_X_CM, START_Y_CM, START_HEADING_RAD)
                    ekf.lost_count = 0
                    ekf.recent_innovations.clear()
                    print(f" Re-acquired: Δθ={sweep_info['delta_theta_deg']}°")
                except Exception as e:
                    print(f" Re-acquire failed: {e}")

            x, y, theta = ekf.get_pose()

            if _x_correction_active:
                xc_elapsed = now - _x_correction_start
                if xc_elapsed > _x_correction_timeout:
                    _x_correction_active = False
                elif d is not None and d < BRAIN_CFG["OBSTACLE_THRESHOLD_CM"]:
                    _x_correction_active = False
                elif ekf.x_corrections > _x_corrections_before:
                    _x_correction_active = False
                else:
                    x_walls = [w for w in wall_map.walls if abs(w.A) > 0.9]
                    x_wall = min(x_walls, key=lambda w: abs(
                        heading_error(w.normal_angle, theta)))
                    err = heading_error(x_wall.normal_angle, theta)
                    if abs(err) > 0.26:
                        td = 1 if err > 0 else -1
                        ts = BRAIN_CFG["TURN_SPEED"]
                        ls, rs = -ts * td, ts * td
                    else:
                        ls, rs = BRAIN_CFG["EXPLORE_SPEED"], BRAIN_CFG["EXPLORE_SPEED"]
                    drive.drive_speeds(ls, rs)
                    ll, lr = ls, rs
            else:
                ls, rs = brain.decide(dl, df, dr, x, y, theta, dt)

                if ekf.needs_x_correction():
                    no_obstacle = d is None or d > BRAIN_CFG["OBSTACLE_THRESHOLD_CM"]
                    not_avoiding = brain.state != State.AVOID
                    if no_obstacle and not_avoiding:
                        _x_correction_active = True
                        _x_correction_start = now
                        _x_corrections_before = ekf.x_corrections
                        print("\n  → aligning to side wall for x-correction...")

            if not _x_correction_active:
                drive.drive_speeds(ls, rs)
                ll, lr = ls, rs

            if on_cycle:
                data = dict(
                    d=d, dl=dl, df=df, dr=dr,
                    x=x, y=y, theta=theta, ls=ls, rs=rs,
                    brain=brain, perim=perim, walls=wall_map.to_dict(),
                )
                on_cycle(data)

            now2 = time.time()
            if now2 - last_log[0] >= 0.5:
                last_log[0] = now2
                d_str = f"{d:5.0f}" if d is not None else "  ---"
                dls = f"{dl:4.0f}" if dl is not None else " ---"
                dfs = f"{df:4.0f}" if df is not None else " ---"
                drs = f"{dr:4.0f}" if dr is not None else " ---"
                inside = perim.is_inside(x, y)
                edge = perim.distance_to_edge(x, y)
                flag = ""
                if not inside:
                    flag = " !OUTSIDE!"
                elif edge < BRAIN_CFG["PERIMETER_MARGIN_CM"]:
                    flag = " ~edge"

                label = _state_label(brain)
                print(
                    f"[{label:>14}] "
                    f"x={x:7.1f} y={y:7.1f} "
                    f"θ={math.degrees(theta):6.1f}° "
                    f"sL={dls} sF={dfs} sR={drs} min={d_str}cm "
                    f"L={ls:+.2f} R={rs:+.2f}"
                    f"{flag}"
                )

            time.sleep(1.0 / BRAIN_CFG["LOOP_HZ"])
    except Exception as e:
        print(f"Nav error: {e}")
    finally:
        drive.stop()
        drive.cleanup()
        for s in sonars:
            s.cleanup()
        GPIO.cleanup()


def main():
    run_navigation(on_cycle=None)


if __name__ == "__main__":
    main()
