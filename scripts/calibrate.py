"""ESC calibration wizard — run ONCE before first use.

Usage:
    python -m scripts.calibrate
"""

from motors.drive import DuckDrive

LEFT_PIN = 12
RIGHT_PIN = 13


def main():
    print("DuckBot — ESC Calibration")
    print(f"  Left ESC  → GPIO {LEFT_PIN}")
    print(f"  Right ESC → GPIO {RIGHT_PIN}")

    drive = DuckDrive(LEFT_PIN, RIGHT_PIN)
    try:
        drive.calibrate()
    finally:
        drive.cleanup()


if __name__ == "__main__":
    main()
