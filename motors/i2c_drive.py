import struct
import fcntl
import os
import logging

from navigation.config import I2C_BUS, I2C_ADDR, I2C_REG, PWM_MIN, PWM_MAX

log = logging.getLogger(__name__)

I2C_SLAVE = 0x0703
I2C_TIMEOUT = 0x0706
I2C_RETRIES = 0x0701
I2C_FUNCS = 0x0705


class I2CDrive:
    def __init__(self, bus=I2C_BUS, address=I2C_ADDR, register=I2C_REG):
        self._address = address
        self._register = register
        self._fd = None
        try:
            self._fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
            fcntl.ioctl(self._fd, I2C_TIMEOUT, 2)
            fcntl.ioctl(self._fd, I2C_RETRIES, 1)
            fcntl.ioctl(self._fd, I2C_SLAVE, address)
        except Exception as e:
            log.warning("I2C unavailable (%s) — mock mode", e)
            self._fd = None

    def drive_speeds(self, left_speed, right_speed):
        def pwm(speed):
            if speed <= 0:
                return PWM_MIN
            return round(PWM_MIN + speed * (PWM_MAX - PWM_MIN))

        payload = bytes([self._register]) + list(struct.pack(">HH", pwm(left_speed), pwm(right_speed)))
        if self._fd is None:
            return
        try:
            os.write(self._fd, payload)
        except OSError as e:
            log.error("I2C write failed: %s", e)

    def stop(self):
        self.drive_speeds(0, 0)

    def cleanup(self):
        self.stop()
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
