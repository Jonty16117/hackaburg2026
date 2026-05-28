import RPi.GPIO as GPIO

PWM_FREQ = 50
PULSE_MIN = 1000
PULSE_NEUTRAL = 1500
PULSE_MAX = 2000


def _us_to_duty(us):
    return (us / 1_000_000) * PWM_FREQ * 100


class Motor:
    def __init__(self, signal_pin):
        self.signal_pin = signal_pin
        self._current_us = PULSE_NEUTRAL
        GPIO.setup(signal_pin, GPIO.OUT)
        self._pwm = GPIO.PWM(signal_pin, PWM_FREQ)
        self._pwm.start(_us_to_duty(PULSE_NEUTRAL))

    def set_pulse(self, us):
        us = max(PULSE_MIN, min(PULSE_MAX, int(us)))
        self._current_us = us
        self._pwm.ChangeDutyCycle(_us_to_duty(us))

    def stop(self):
        self.set_pulse(PULSE_NEUTRAL)

    def cleanup(self):
        self._pwm.stop()
