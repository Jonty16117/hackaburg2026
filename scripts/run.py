"""Scripted / interactive motor control for DuckBot.

Usage:
    python -m scripts.run              # choose mode interactively
    python -m scripts.run auto         # run auto test sequence
    python -m scripts.run manual       # interactive keyboard control
    python -m scripts.run calibrate    # ESC calibration wizard
"""

import sys
import time
import RPi.GPIO as GPIO
from motors.drive import DuckDrive
from navigation.config import LEFT_PIN, RIGHT_PIN


def auto_sequence(drive):
    print("\n=== AUTO TEST SEQUENCE ===")
    print("Press Ctrl+C to abort.\n")

    steps = [
        ("Forward 50%", lambda: drive.forward(0.5), 3),
        ("Stop", drive.stop, 1),
        ("Left turn 50%", lambda: drive.turn_left(0.5), 2),
        ("Right turn 50%", lambda: drive.turn_right(0.5), 2),
        ("Stop", drive.stop, 0),
    ]

    try:
        for i, (label, action, duration) in enumerate(steps, 1):
            print(f"[{i}/{len(steps)}] {label}...")
            action()
            if duration:
                time.sleep(duration)
        print("Done!")
    except KeyboardInterrupt:
        print("\nAborted.")
        drive.stop()


def manual_control(drive):
    print("\nManual control")
    print("  w = forward  a = left  d = right")
    print("  q = stop  x = exit")
    print("  1-9 = speed (1=10%, 9=90%)\n")

    speed = 0.5
    while True:
        cmd = input("> ").strip().lower()
        if cmd == "w":
            drive.forward(speed)
            print(f"  forward @ {int(speed * 100)}%")
        elif cmd == "a":
            drive.turn_left(speed)
            print(f"  left @ {int(speed * 100)}%")
        elif cmd == "d":
            drive.turn_right(speed)
            print(f"  right @ {int(speed * 100)}%")
        elif cmd == "q":
            drive.stop()
            print("  stop")
        elif cmd == "x":
            break
        elif cmd.isdigit():
            speed = int(cmd) / 10
            speed = max(0.1, min(1.0, speed))
            print(f"  speed -> {int(speed * 100)}%")
        else:
            print(f"  unknown: {cmd}")


def calibrate(drive):
    print("\n=== ESC CALIBRATION ===")
    drive.calibrate()


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    mode = sys.argv[1] if len(sys.argv) > 1 else None

    print("DuckBot — Motor Control")
    print(f"  Left ESC  -> GPIO {LEFT_PIN}")
    print(f"  Right ESC -> GPIO {RIGHT_PIN}")

    drive = DuckDrive(LEFT_PIN, RIGHT_PIN)

    try:
        if mode == "auto":
            auto_sequence(drive)
        elif mode == "manual":
            manual_control(drive)
        elif mode == "calibrate":
            calibrate(drive)
        else:
            m = input("\nMode: [a]uto  [m]anual  [c]alibrate  [q]uit: ").strip().lower()
            if m == "a":
                auto_sequence(drive)
            elif m == "m":
                manual_control(drive)
            elif m == "c":
                calibrate(drive)
    finally:
        drive.cleanup()
        print("GPIO cleaned up.")


if __name__ == "__main__":
    main()
