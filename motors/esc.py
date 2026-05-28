import RPi.GPIO as GPIO
import time
import threading

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

PWM_FREQ = 50   # Hz — standard ESC signal
PULSE_MIN = 1000  # µs — full reverse / brake
PULSE_NEUTRAL = 1500  # µs — stop
PULSE_MAX = 2000  # µs — full forward


def _us_to_duty(us):
    """Convert microsecond pulse width to duty cycle (0-100)."""
    return (us / 1_000_000) * PWM_FREQ * 100


class Motor:
    def __init__(self, signal_pin, name="motor"):
        self.name = name
        self.signal_pin = signal_pin
        self._running = False
        self._current_us = PULSE_NEUTRAL

        GPIO.setup(signal_pin, GPIO.OUT)
        self._pwm = GPIO.PWM(signal_pin, PWM_FREQ)
        self._pwm.start(_us_to_duty(PULSE_NEUTRAL))

    def set_pulse(self, us):
        """Set throttle in microseconds (1000-2000)."""
        us = max(PULSE_MIN, min(PULSE_MAX, int(us)))
        self._current_us = us
        self._pwm.ChangeDutyCycle(_us_to_duty(us))

    def stop(self):
        """Stop the motor (neutral)."""
        self.set_pulse(PULSE_NEUTRAL)

    def forward(self, speed=1.0):
        """Forward at given speed (0.0 to 1.0)."""
        pulse = PULSE_NEUTRAL + speed * (PULSE_MAX - PULSE_NEUTRAL)
        self.set_pulse(pulse)

    def reverse(self, speed=1.0):
        """Reverse at given speed (0.0 to 1.0)."""
        pulse = PULSE_NEUTRAL - speed * (PULSE_NEUTRAL - PULSE_MIN)
        self.set_pulse(pulse)

    def cleanup(self):
        self._pwm.stop()


class DuckDrive:
    """Differential drive for two motors."""

    def __init__(self, left_pin, right_pin):
        self.left = Motor(left_pin, name="left")
        self.right = Motor(right_pin, name="right")

    def stop(self):
        self.left.stop()
        self.right.stop()

    def forward(self, speed=1.0):
        self.left.forward(speed)
        self.right.forward(speed)

    def reverse(self, speed=1.0):
        self.left.reverse(speed)
        self.right.reverse(speed)

    def turn_left(self, speed=1.0):
        self.left.reverse(speed)
        self.right.forward(speed)

    def turn_right(self, speed=1.0):
        self.left.forward(speed)
        self.right.reverse(speed)

    def set_raw(self, left_us, right_us):
        self.left.set_pulse(left_us)
        self.right.set_pulse(right_us)

    def cleanup(self):
        self.left.cleanup()
        self.right.cleanup()
        GPIO.cleanup()

    def calibrate(self):
        """ESC calibration sequence — run ONCE per ESC.

        1. Send max pulse (2000µs)
        2. Power on ESC
        3. Wait for beep
        4. Send min pulse (1000µs)
        5. Wait for confirmation beeps
        6. Send neutral (1500µs)
        """
        print("=== ESC CALIBRATION ===")
        print("1. DISCONNECT the battery from ESCs.")
        print("2. Press Enter to send max signal (2000µs)...")
        input()
        self.left.set_pulse(PULSE_MAX)
        self.right.set_pulse(PULSE_MAX)
        print("Sent 2000µs on both ESCs.")

        print("3. CONNECT the battery to ESCs NOW.")
        print("   Wait for a beep from both ESCs, then press Enter...")
        input()
        self.left.set_pulse(PULSE_MIN)
        self.right.set_pulse(PULSE_MIN)
        print("Sent 1000µs. Waiting for confirmation beeps...")
        time.sleep(2)

        print("4. Sending neutral (1500µs)...")
        self.left.set_pulse(PULSE_NEUTRAL)
        self.right.set_pulse(PULSE_NEUTRAL)
        time.sleep(1)
        print("Calibration complete! ESCs ready.")
