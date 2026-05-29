"""Scripted / interactive motor control for DuckBot.

Usage:
    python -m scripts.run              # choose mode interactively
    python -m scripts.run auto         # run auto test sequence
    python -m scripts.run manual       # interactive keyboard control
"""

import sys
import time
from motors.i2c_drive import I2CDrive


def _stop(drive):
    drive.drive_speeds(0, 0)


def auto_sequence(drive):
    print("\n=== AUTO TEST SEQUENCE ===")
    print("Press Ctrl+C to abort.\n")

    steps = [
        ("Forward 20%", lambda: drive.drive_speeds(0.2, 0.2), 3),
        ("Stop", lambda: _stop(drive), 1),
        ("Left pivot", lambda: drive.drive_speeds(0, 0.3), 2),
        ("Right pivot", lambda: drive.drive_speeds(0.3, 0), 2),
        ("Forward 40%", lambda: drive.drive_speeds(0.4, 0.4), 3),
        ("Stop", lambda: _stop(drive), 0),
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
        _stop(drive)


def manual_control(drive):
    print("\nManual control  (forward only, no reverse)")
    print("  w = forward   a = left pivot   d = right pivot")
    print("  q = stop   x = exit")
    print("  1-9 = speed (1=10%, 9=90%)\n")

    speed = 0.3
    while True:
        cmd = input("> ").strip().lower()
        if cmd == "w":
            drive.drive_speeds(speed, speed)
            print(f"  forward @ {int(speed * 100)}%")
        elif cmd == "a":
            drive.drive_speeds(0, speed)
            print(f"  left pivot @ {int(speed * 100)}%")
        elif cmd == "d":
            drive.drive_speeds(speed, 0)
            print(f"  right pivot @ {int(speed * 100)}%")
        elif cmd == "q":
            _stop(drive)
            print("  stop")
        elif cmd == "x":
            break
        elif cmd.isdigit():
            speed = int(cmd) / 10
            speed = max(0.1, min(1.0, speed))
            print(f"  speed -> {int(speed * 100)}%")
        else:
            print(f"  unknown: {cmd}")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else None

    print("DuckBot — Motor Control (I2C)")
    drive = I2CDrive()

    try:
        if mode == "auto":
            auto_sequence(drive)
        elif mode == "manual":
            manual_control(drive)
        else:
            m = input("\nMode: [a]uto  [m]anual  [q]uit: ").strip().lower()
            if m == "a":
                auto_sequence(drive)
            elif m == "m":
                manual_control(drive)
    finally:
        drive.cleanup()


if __name__ == "__main__":
    main()
