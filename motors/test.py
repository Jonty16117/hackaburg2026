"""Motor test script for DuckBot.

Wiring:
    Left ESC signal  → GPIO 12 (BCM)
    Right ESC signal → GPIO 13 (BCM)
    ESC GND (brown)  → Pi GND (pin 6, 9, 14, etc.)
    ESC 5V (red)     → DO NOT CONNECT (Pi powered via USB)

Usage:
    python -m motors.test          # interactive mode
    python -m motors.test calibrate  # calibrate ESCs first
"""

import sys
import time
from motors.esc import DuckDrive, PULSE_MIN, PULSE_NEUTRAL, PULSE_MAX

LEFT_PIN = 12
RIGHT_PIN = 13


def manual_control(drive):
    print("\nManual control: w=forward s=reverse a=left d=right q=stop x=exit")
    print("Speed steps: 1-9 (1=slow, 9=fast)\n")

    speed = 0.5
    while True:
        cmd = input("> ").strip()
        if cmd == "w":
            drive.forward(speed)
            print(f"  forward @ {speed:.0%}")
        elif cmd == "s":
            drive.reverse(speed)
            print(f"  reverse @ {speed:.0%}")
        elif cmd == "a":
            drive.turn_left(speed)
            print(f"  left turn @ {speed:.0%}")
        elif cmd == "d":
            drive.turn_right(speed)
            print(f"  right turn @ {speed:.0%}")
        elif cmd == "q":
            drive.stop()
            print("  stop")
        elif cmd == "x":
            break
        elif cmd.isdigit():
            speed = int(cmd) / 10
            speed = max(0.1, min(1.0, speed))
            print(f"  speed set to {speed:.0%}")
        else:
            print("  ?")


def auto_test(drive):
    print("\n=== AUTO TEST SEQUENCE ===")
    print("Motors will run through forward, turns, reverse, stop.")
    print("Press Ctrl+C to abort.\n")

    try:
        print("[1/6] Forward 50% for 3s...")
        drive.forward(0.5)
        time.sleep(3)

        print("[2/6] Stop for 1s...")
        drive.stop()
        time.sleep(1)

        print("[3/6] Left turn 50% for 2s...")
        drive.turn_left(0.5)
        time.sleep(2)

        print("[4/6] Right turn 50% for 2s...")
        drive.turn_right(0.5)
        time.sleep(2)

        print("[5/6] Reverse 50% for 3s...")
        drive.reverse(0.5)
        time.sleep(3)

        print("[6/6] Stop.")
        drive.stop()

        print("Done!")
    except KeyboardInterrupt:
        print("\nAborted.")
        drive.stop()


if __name__ == "__main__":
    print("DuckBot Motor Test")
    print(f"  Left ESC  → GPIO {LEFT_PIN}")
    print(f"  Right ESC → GPIO {RIGHT_PIN}")
    print(f"  Pulse range: {PULSE_MIN}-{PULSE_MAX}µs (neutral={PULSE_NEUTRAL}µs)")

    drive = DuckDrive(LEFT_PIN, RIGHT_PIN)

    try:
        if "calibrate" in sys.argv:
            drive.calibrate()

        mode = input("\nMode: [a]uto test  [m]anual control  [q]uit: ").strip().lower()
        if mode == "a":
            auto_test(drive)
        elif mode == "m":
            manual_control(drive)
        else:
            print("Bye.")
    finally:
        drive.cleanup()
