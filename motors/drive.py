import time
import RPi.GPIO as GPIO
from motors.esc import Motor, PULSE_MIN, PULSE_NEUTRAL, PULSE_MAX


class DuckDrive:
    def __init__(self, left_pin, right_pin):
        self.left = Motor(left_pin)
        self.right = Motor(right_pin)

        # Swap any two motor wires if mounted mirrored:
        # True = reverse motor direction vs default.
        self.left_invert = False
        self.right_invert = True  # mirrored propellers

    def _invert(self, pulse, inverted):
        if inverted:
            return PULSE_NEUTRAL - (pulse - PULSE_NEUTRAL)
        return pulse

    def stop(self):
        self.left.set_pulse(PULSE_NEUTRAL)
        self.right.set_pulse(PULSE_NEUTRAL)

    def forward(self, speed=1.0):
        pulse = PULSE_NEUTRAL + speed * (PULSE_MAX - PULSE_NEUTRAL)
        self.left.set_pulse(self._invert(pulse, self.left_invert))
        self.right.set_pulse(self._invert(pulse, self.right_invert))

    def reverse(self, speed=1.0):
        self.stop()

    def turn_left(self, speed=1.0):
        rev = PULSE_NEUTRAL - speed * (PULSE_NEUTRAL - PULSE_MIN)
        fwd = PULSE_NEUTRAL + speed * (PULSE_MAX - PULSE_NEUTRAL)
        self.left.set_pulse(self._invert(rev, self.left_invert))
        self.right.set_pulse(self._invert(fwd, self.right_invert))

    def turn_right(self, speed=1.0):
        rev = PULSE_NEUTRAL - speed * (PULSE_NEUTRAL - PULSE_MIN)
        fwd = PULSE_NEUTRAL + speed * (PULSE_MAX - PULSE_NEUTRAL)
        self.left.set_pulse(self._invert(fwd, self.left_invert))
        self.right.set_pulse(self._invert(rev, self.right_invert))

    def set_raw(self, left_us, right_us):
        self.left.set_pulse(left_us)
        self.right.set_pulse(right_us)

    def drive_speeds(self, left_speed, right_speed):
        ls = max(-1.0, min(1.0, left_speed))
        rs = max(-1.0, min(1.0, right_speed))
        if ls < 0 and rs < 0:
            ls = rs = 0.0
        lp = int(PULSE_NEUTRAL + ls * (PULSE_MAX - PULSE_NEUTRAL))
        rp = int(PULSE_NEUTRAL + rs * (PULSE_MAX - PULSE_NEUTRAL))
        self.set_raw(self._invert(lp, self.left_invert),
                     self._invert(rp, self.right_invert))

    def cleanup(self):
        self.left.cleanup()
        self.right.cleanup()
        GPIO.cleanup()

    def calibrate(self):
        print("=== ESC CALIBRATION ===")
        print("1. DISCONNECT battery from ESCs.")
        print("2. Press Enter to send max signal (2000µs)...")
        input()
        self.left.set_pulse(PULSE_MAX)
        self.right.set_pulse(PULSE_MAX)
        print("Sent 2000µs on both ESCs.")

        print("3. CONNECT battery to ESCs NOW.")
        print("   Wait for the first beep from both, then press Enter...")
        input()
        self.left.set_pulse(PULSE_MIN)
        self.right.set_pulse(PULSE_MIN)
        print("Sent 1000µs. Waiting for confirmation beeps...")
        time.sleep(3)

        print("4. Sending neutral (1500µs)...")
        self.left.set_pulse(PULSE_NEUTRAL)
        self.right.set_pulse(PULSE_NEUTRAL)
        time.sleep(1)
        print("Calibration complete!")
