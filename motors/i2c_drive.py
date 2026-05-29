import struct
import fcntl
import logging

from navigation.config import I2C_BUS, I2C_ADDR, I2C_REG, PWM_MIN, PWM_MAX

log = logging.getLogger(__name__)

I2C_TIMEOUT = 0x0706


class I2CDrive:
    def __init__(self, bus=I2C_BUS, address=I2C_ADDR, register=I2C_REG):
        self._address = address
        self._register = register
        self._bus = None
        try:
            import smbus2
            self._bus = smbus2.SMBus(bus)
            fcntl.ioctl(self._bus.fd, I2C_TIMEOUT, 2)
        except Exception as e:
            log.warning("I2C unavailable (%s) — mock mode", e)

    def drive_speeds(self, left_speed, right_speed):
        def pwm(speed):
            if speed <= 0:
                return PWM_MIN
            return round(PWM_MIN + speed * (PWM_MAX - PWM_MIN))

        payload = list(struct.pack(">HH", pwm(left_speed), pwm(right_speed)))
        if self._bus is None:
            return
        try:
            self._bus.write_i2c_block_data(self._address, self._register, payload)
        except OSError as e:
            log.error("I2C write failed: %s", e)

    def stop(self):
        self.drive_speeds(0, 0)

    def cleanup(self):
        self.stop()
        if self._bus is not None:
            try:
                self._bus.close()
            except OSError:
                pass
