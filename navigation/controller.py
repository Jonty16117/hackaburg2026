"""DuckBot autonomous navigation controller.

Reads sonar, dead-reckons position, runs the brain state machine,
and sends motor commands at ~20 Hz.

Usage:
    python -m navigation.controller
"""

import math
import time

from navigation.config import (
    LEFT_PIN, RIGHT_PIN, SONAR_TRIG, SONAR_ECHO,
    PERIMETER_CM, START_X_CM, START_Y_CM, START_HEADING_RAD,
    BRAIN_CFG,
)
from navigation.odometry import Odometry
from navigation.perimeter import Perimeter
from navigation.brain import Brain, State, _AvoidPhase


AVOID_PHASE_LABELS = {
    _AvoidPhase.REVERSE:         "AVOID:REV",
    _AvoidPhase.SCAN_LEFT:       "AVOID:SCAN_L",
    _AvoidPhase.SCAN_LEFT_READ:  "AVOID:READ_L",
    _AvoidPhase.SCAN_RIGHT:      "AVOID:SCAN_R",
    _AvoidPhase.SCAN_RIGHT_READ: "AVOID:READ_R",
    _AvoidPhase.COMPLETE_TURN:   "AVOID:TURN",
    _AvoidPhase.DONE:            "AVOID:DONE",
}


def _state_label(brain):
    if brain.state == State.AVOID:
        return AVOID_PHASE_LABELS.get(brain._avoid_phase, "AVOID")
    return brain.state.name


def run_navigation(on_cycle=None):
    """Run the sense→think→act navigation loop at ~20 Hz.

    Args:
        on_cycle: optional callback(dict) called each iteration with
                  keys: d, x, y, theta, ls, rs, brain, perim
    """
    import RPi.GPIO as GPIO
    from motors.drive import DuckDrive
    from sensors.sonar import Sonar

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    drive = DuckDrive(LEFT_PIN, RIGHT_PIN)
    sonar = Sonar(SONAR_TRIG, SONAR_ECHO)
    odom = Odometry(START_X_CM, START_Y_CM, START_HEADING_RAD,
                    BRAIN_CFG["WHEEL_BASE_CM"], BRAIN_CFG["MAX_SPEED_CM_S"])
    perim = Perimeter(PERIMETER_CM)
    brain = Brain(perim, BRAIN_CFG)

    ll, lr = 0.0, 0.0
    lt = time.time()

    try:
        while True:
            now = time.time()
            dt = max(now - lt, 0.001)
            lt = now

            odom.update(ll, lr, dt)
            d = sonar.distance_cm()
            x, y, theta = odom.position()
            ls, rs = brain.decide(d, x, y, theta, dt)
            drive.drive_speeds(ls, rs)
            ll, lr = ls, rs

            if on_cycle:
                on_cycle(dict(
                    d=d, x=x, y=y, theta=theta, ls=ls, rs=rs,
                    brain=brain, perim=perim,
                ))

            time.sleep(1.0 / BRAIN_CFG["LOOP_HZ"])
    except Exception as e:
        print(f"Nav error: {e}")
    finally:
        drive.stop()
        drive.cleanup()
        sonar.cleanup()
        GPIO.cleanup()


def main():
    print("=" * 72)
    print("DuckBot — Autonomous Navigation Controller")
    print(f"  Perimeter : {PERIMETER_CM[2][0]} \u00d7 {PERIMETER_CM[2][1]} cm")
    print(f"  Start pose: ({START_X_CM}, {START_Y_CM})  "
          f"@{math.degrees(START_HEADING_RAD):.0f}\u00b0")
    print(f"  Sonar     : TRIG=GPIO{SONAR_TRIG}  ECHO=GPIO{SONAR_ECHO}")
    print(f"  Motors    : LEFT=GPIO{LEFT_PIN}  RIGHT=GPIO{RIGHT_PIN}")
    print(f"  Loop rate : {BRAIN_CFG['LOOP_HZ']} Hz")
    print("  Press Ctrl+C to stop.")
    print("=" * 72)

    last_log = [time.time()]

    def on_cycle(data):
        now = time.time()
        if now - last_log[0] < 0.5:
            return
        last_log[0] = now

        d = data["d"]
        dist_str = f"{d:5.0f}" if d is not None else "  ---"
        inside = data["perim"].is_inside(data["x"], data["y"])
        edge = data["perim"].distance_to_edge(data["x"], data["y"])
        flag = ""
        if not inside:
            flag = " !OUTSIDE!"
        elif edge < BRAIN_CFG["PERIMETER_MARGIN_CM"]:
            flag = " ~edge"

        label = _state_label(data["brain"])
        print(
            f"[{label:>14}] "
            f"x={data['x']:7.1f} y={data['y']:7.1f} "
            f"\u03b8={math.degrees(data['theta']):6.1f}\u00b0 "
            f"sonar={dist_str}cm "
            f"L={data['ls']:+.2f} R={data['rs']:+.2f}"
            f"{flag}"
        )

    run_navigation(on_cycle=on_cycle)


if __name__ == "__main__":
    main()
