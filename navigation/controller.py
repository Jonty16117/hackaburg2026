"""DuckBot autonomous navigation controller.

Runs boot-time wall orientation sweep, tracks position via EKF,
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
    MAPPER_BLIND_ZONE_CM,
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

    wall_map = WallMap()
    ekf = None
    sweep_info = None

    if MAPPER_TOGGLE:
        print("=" * 60)
        print(" Wall orientation sweep running... (~18s)")
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

    perim = wall_map.to_perimeter()
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
            ekf.predict(ll, lr, dt)

            d = sonar.distance_cm()
            if d is not None:
                ekf.correct(d)

            if ekf.should_reacquire():
                print("\n EKF lost while idle — re-running sweep...")
                try:
                    sweeper = SonarSweep(
                        drive, sonar,
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

            ls, rs = brain.decide(d, x, y, theta, dt)

            if ekf.needs_x_correction():
                x_wall_normal = wall_map.walls[0].normal_angle
                err = x_wall_normal - theta
                err = math.atan2(math.sin(err), math.cos(err))
                bias = math.copysign(0.05, err)
                ls = min(1.0, max(-1.0, ls + bias * 0.3))
                rs = min(1.0, max(-1.0, rs - bias * 0.3))

            drive.drive_speeds(ls, rs)
            ll, lr = ls, rs

            if on_cycle:
                data = dict(
                    d=d, x=x, y=y, theta=theta, ls=ls, rs=rs,
                    brain=brain, perim=perim, walls=wall_map.to_dict(),
                )
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

                label = _state_label(brain)
                print(
                    f"[{label:>14}] "
                    f"x={x:7.1f} y={y:7.1f} "
                    f"θ={math.degrees(theta):6.1f}° "
                    f"sonar={d_str}cm "
                    f"L={ls:+.2f} R={rs:+.2f}"
                    f"{flag}"
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
