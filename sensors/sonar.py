import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

SPEED_OF_SOUND = 34300  # cm/s at 20°C


class Sonar:
    def __init__(self, trig_pin=23, echo_pin=24):
        self.trig = trig_pin
        self.echo = echo_pin
        GPIO.setup(trig_pin, GPIO.OUT)
        GPIO.setup(echo_pin, GPIO.IN)
        GPIO.output(trig_pin, GPIO.LOW)
        time.sleep(0.1)

    def distance_cm(self, timeout=0.04):
        GPIO.output(self.trig, GPIO.HIGH)
        time.sleep(0.000_01)  # 10µs pulse
        GPIO.output(self.trig, GPIO.LOW)

        pulse_start = time.time()
        pulse_end = time.time()

        t0 = time.time()
        while GPIO.input(self.echo) == GPIO.LOW:
            pulse_start = time.time()
            if pulse_start - t0 > timeout:
                return None

        t0 = time.time()
        while GPIO.input(self.echo) == GPIO.HIGH:
            pulse_end = time.time()
            if pulse_end - t0 > timeout:
                return None

        duration = pulse_end - pulse_start
        return (duration * SPEED_OF_SOUND) / 2.0

    def cleanup(self):
        GPIO.cleanup([self.trig, self.echo])


def demo():
    s = Sonar()
    print("RCWL-1655 Sonar Demo — press Ctrl+C to stop")
    try:
        while True:
            d = s.distance_cm()
            if d is None:
                print("  out of range")
            else:
                print(f"  {d:6.1f} cm")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nDone.")
    finally:
        s.cleanup()


if __name__ == "__main__":
    demo()
