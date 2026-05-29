"""DuckBot autonomous navigation controller.

Reads sonar, runs boot-time wall sweep, tracks position via EKF,
and sends motor commands at ~20 Hz.

Usage:
    python -m navigation.controller
"""

import math
import time

from navigation.config import (
    LEFT_PIN, RIGHT_PIN, SONAR_TRIG, SONAR_ECHO,
    PERIMETER_CM, START_X_CM, START_Y_CM, START_HEADING_RAD,
    END_X_CM, END_Y_CM, BRAIN_CFG, MAPPER_TOGGLE,
    MAPPER_SWEEP_SPEED, MAPPER_SWEEP_DEG_STEP, MAPPER_SONAR_SAMPLES,
    MAPPER_BLIND_ZONE_CM, EKF_LOCK_COVARIANCE, EKF_LOCK_MIN_OBS,
)
from navigation.odometry import Odometry
from navigation.perimeter import Perimeter
from navigation.brain import Brain, State, _AvoidPhase
from navigation.wall_map import WallMap
from navigation.ekf_localizer import EKFLocalizer


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


def run_navigation(on_cycle=None):
    import RPi.GPIO as GPIO
    from motors.drive import DuckDrive
    from sensors.sonar import Sonar
    from navigation.sonar_sweep import SonarSweep

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    drive = DuckDrive(LEFT_PIN, RIGHT_PIN)
    sonar = Sonar(SONAR_TRIG, SONAR_ECHO)
    odom = Odometry(START_X_CM, START_Y_CM, START_HEADING_RAD,
                    BRAIN_CFG["WHEEL_BASE_CM"], BRAIN_CFG["MAX_SPEED_CM_S"])

    wall_map = None
    ekf = None
    sweep_result = None

    if MAPPER_TOGGLE:
        print("=" * 60)
        print(" Wall mapping sweep running... (~18s)")
        print("=" * 60)
        try:
            sweeper = SonarSweep(
                drive, sonar,
                sweep_speed=MAPPER_SWEEP_SPEED,
                step_deg=MAPPER_SWEEP_DEG_STEP,
                sonar_samples=MAPPER_SONAR_SAMPLES,
                blind_zone_cm=MAPPER_BLIND_ZONE_CM,
            )
            sweep_result = sweeper.run(odom)
            wall_map = WallMap()
            wall_map.init_from_minima(
                sweep_result["readings"],
                duck_x=sweep_result["duck_x"],
                duck_y=sweep_result["duck_y"],
            )
            odom.x = sweep_result["duck_x"]
            odom.y = sweep_result["duck_y"]
            odom.theta = 0.0

            n_walls = len(wall_map.walls)
            print(f"\n Sweep complete: {n_walls}/4 walls detected.")
            for i, w in enumerate(wall_map.walls):
                print(f"  Wall {i}: normal={math.degrees(w.normal_angle):.0f}°  "
                      f"C={w.C:.1f}")
            if n_walls < 4:
                print(" WARNING: fewer than 4 walls found — "
                      "localization will be degraded.")
        except Exception as e:
            print(f" Sweep failed: {e}")
            print(" Falling back to hardcoded perimeter.")
            wall_map = None

    if wall_map is None:
        perim = Perimeter(PERIMETER_CM)
    else:
        perim = wall_map.to_perimeter()

    brain = Brain(perim, BRAIN_CFG, goal_x=END_X_CM, goal_y=END_Y_CM)

    if wall_map is not None:
        ekf = EKFLocalizer(
            wall_map,
            (START_X_CM, START_Y_CM, START_HEADING_RAD),
            wheel_base_cm=BRAIN_CFG["WHEEL_BASE_CM"],
            max_speed_cm_s=BRAIN_CFG["MAX_SPEED_CM_S"],
        )

    print("=" * 72)
    print("DuckBot — Autonomous Navigation Controller")
    if wall_map is not None:
        if wall_map.corners:
            print(f"  Mapped pool : {len(wall_map.corners)} corners "
                  f"(phase: {wall_map.phase})")
    else:
        print(f"  Perimeter   : {PERIMETER_CM[2][0]} x {PERIMETER_CM[2][1]} cm")
    print(f"  Start pose  : ({START_X_CM}, {START_Y_CM})  "
          f"@{math.degrees(START_HEADING_RAD):.0f}°")
    print(f"  Sonar       : TRIG=GPIO{SONAR_TRIG}  ECHO=GPIO{SONAR_ECHO}")
    print(f"  Motors      : LEFT=GPIO{LEFT_PIN}  RIGHT=GPIO{RIGHT_PIN}")
    print(f"  Loop rate   : {BRAIN_CFG['LOOP_HZ']} Hz")
    print("  Press Ctrl+C to stop.")
    print("=" * 72)

    ll, lr = 0.0, 0.0
    lt = time.time()
    last_log = [lt]

    try:
        while True:
            now = time.time()
            dt = max(now - lt, 0.001)
            lt = now

            odom.update(ll, lr, dt)

            if ekf is not None:
                ekf.predict(ll, lr, dt)

            d = sonar.distance_cm()

            if ekf is not None:
                if d is not None:
                    ekf.correct(d)

                if ekf.should_reacquire():
                    print("\n EKF lost — re-acquiring walls...")
                    try:
                        sweeper = SonarSweep(
                            drive, sonar,
                            sweep_speed=MAPPER_SWEEP_SPEED,
                            step_deg=MAPPER_SWEEP_DEG_STEP,
                            sonar_samples=MAPPER_SONAR_SAMPLES,
                            blind_zone_cm=MAPPER_BLIND_ZONE_CM,
                        )
                        result = sweeper.run(odom)
                        wall_map.init_from_minima(
                            result["readings"],
                            duck_x=result["duck_x"],
                            duck_y=result["duck_y"],
                        )
                        ekf.set_pose(
                            result["duck_x"],
                            result["duck_y"],
                            0.0,
                        )
                        ekf.lost_count = 0
                        ekf.recent_innovations.clear()
                        perim = wall_map.to_perimeter()
                        brain.perimeter = perim
                        print(f" Re-acquired: {len(wall_map.walls)} walls "
                              f"@ ({result['duck_x']:.0f}, "
                              f"{result['duck_y']:.0f})")
                    except Exception as e:
                        print(f" Re-acquire failed: {e}")

                x, y, theta = ekf.get_pose()
            else:
                x, y, theta = odom.position()

            ls, rs = brain.decide(d, x, y, theta, dt)
            drive.drive_speeds(ls, rs)
            ll, lr = ls, rs

            if on_cycle:
                data = dict(
                    d=d, x=x, y=y, theta=theta, ls=ls, rs=rs,
                    brain=brain, perim=perim,
                )
                if wall_map is not None:
                    data["walls"] = wall_map.to_dict()
                else:
                    data["walls"] = None
                on_cycle(data)

            now2 = time.time()
            if now2 - last_log[0] >= 0.5:
                last_log[0] = now2
                d_str = f"{d:5.0f}" if d is not None else "  ---"
                inside = perim.is_inside(x, y)
                edge = perim.distance_to_edge(x, y)
                flag = ""
                if not inside:
                    flag = " !OUTSIDE!"
                elif edge < BRAIN_CFG["PERIMETER_MARGIN_CM"]:
                    flag = " ~edge"

                wphase = ""
                if wall_map is not None:
                    wphase = f" walls:{wall_map.phase}"
                label = _state_label(brain)
                print(
                    f"[{label:>14}] "
                    f"x={x:7.1f} y={y:7.1f} "
                    f"θ={math.degrees(theta):6.1f}° "
                    f"sonar={d_str}cm "
                    f"L={ls:+.2f} R={rs:+.2f}"
                    f"{flag}{wphase}"
                )

            time.sleep(1.0 / BRAIN_CFG["LOOP_HZ"])
    except Exception as e:
        print(f"Nav error: {e}")
    finally:
        drive.stop()
        drive.cleanup()
        sonar.cleanup()
        GPIO.cleanup()


def main():
    run_navigation(on_cycle=None)


if __name__ == "__main__":
    main()
