"""DuckBot autonomous navigation controller.

Reads sonar, dead-reckons position, runs the brain state machine,
and sends motor commands at ~20 Hz.

Usage:
    python -m navigation.controller
"""

import math
import time
import RPi.GPIO as GPIO

from motors.drive import DuckDrive
from sensors.sonar import Sonar
from navigation.config import (
    LEFT_PIN,
    RIGHT_PIN,
    SONAR_TRIG,
    SONAR_ECHO,
    PERIMETER_CM,
    START_X_CM,
    START_Y_CM,
    START_HEADING_RAD,
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


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    drive = DuckDrive(LEFT_PIN, RIGHT_PIN)
    sonar = Sonar(SONAR_TRIG, SONAR_ECHO)

    odom = Odometry(
        x_cm=START_X_CM,
        y_cm=START_Y_CM,
        theta_rad=START_HEADING_RAD,
        wheel_base_cm=BRAIN_CFG["WHEEL_BASE_CM"],
        max_speed_cm_s=BRAIN_CFG["MAX_SPEED_CM_S"],
    )

    perim = Perimeter(PERIMETER_CM)
    brain = Brain(perim, BRAIN_CFG)

    min_x, max_x, min_y, max_y = perim.bounds()

    print("=" * 72)
    print("DuckBot — Autonomous Navigation Controller")
    print(f"  Perimeter : {PERIMETER_CM[2][0]} × {PERIMETER_CM[2][1]} cm")
    print(f"  Start pose: ({START_X_CM}, {START_Y_CM}) @ {math.degrees(START_HEADING_RAD):.0f}°")
    print(f"  Sonar     : TRIG=GPIO{SONAR_TRIG}  ECHO=GPIO{SONAR_ECHO}")
    print(f"  Motors    : LEFT=GPIO{LEFT_PIN}  RIGHT=GPIO{RIGHT_PIN}")
    print(f"  Loop rate : {BRAIN_CFG['LOOP_HZ']} Hz")
    print("  Press Ctrl+C to stop.")
    print("=" * 72)

    last_frame = time.time()
    last_log = time.time()
    last_left = 0.0
    last_right = 0.0

    try:
        while True:
            now = time.time()
            dt = now - last_frame
            last_frame = now

            odom.update(last_left, last_right, dt)

            distance = sonar.distance_cm()

            x, y, theta = odom.position()
            left_speed, right_speed = brain.decide(distance, x, y, theta, dt)

            drive.drive_speeds(left_speed, right_speed)

            last_left = left_speed
            last_right = right_speed

            if now - last_log > 0.5:
                last_log = now
                dist_str = f"{distance:5.0f}" if distance is not None else "  ---"
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
                    f"sonar={dist_str}cm "
                    f"L={left_speed:+.2f} R={right_speed:+.2f}"
                    f"{flag}"
                )

            time.sleep(1.0 / BRAIN_CFG["LOOP_HZ"])

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        drive.stop()
        drive.cleanup()
        sonar.cleanup()
        GPIO.cleanup()
        print("GPIO cleaned up. Done.")


if __name__ == "__main__":
    main()
