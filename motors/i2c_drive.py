import struct
import fcntl
import os
import logging

logging.basicConfig(level=logging.DEBUG, format="%(message)s")
log = logging.getLogger(__name__)

I2C_BUS = 1
I2C_ADDR = 0x08
PWM_MIN = 1000
PWM_MAX = 1500

I2C_SLAVE = 0x0703
I2C_TIMEOUT = 0x0706
I2C_RETRIES = 0x0701


class I2CDrive:
    def __init__(self, bus=I2C_BUS, address=I2C_ADDR):
        self._address = address
        self._fd = None
        log.info("I2C  bus=%d  addr=0x%02X  range=%d-%d",
                 bus, address, PWM_MIN, PWM_MAX)
        try:
            self._fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
            fcntl.ioctl(self._fd, I2C_TIMEOUT, 2)
            fcntl.ioctl(self._fd, I2C_RETRIES, 1)
            fcntl.ioctl(self._fd, I2C_SLAVE, address)
            log.info("I2C  fd opened OK")
        except Exception as e:
            log.warning("I2C  unavailable (%s) — mock mode", e)
            self._fd = None

    def drive_speeds(self, left_speed, right_speed):
        def pwm(speed):
            if speed <= 0:
                return PWM_MIN
            return round(PWM_MIN + speed * (PWM_MAX - PWM_MIN))

        lp = pwm(left_speed)
        rp = pwm(right_speed)
        payload = struct.pack(">HH", lp, rp)
        hexstr = " ".join(f"{b:02X}" for b in payload)
        log.info("I2C  left=%5.2f→%d  right=%5.2f→%d  raw=[%s]",
                 left_speed, lp, right_speed, rp, hexstr)
        if self._fd is None:
            return
        try:
            os.write(self._fd, payload)
            log.debug("I2C  write OK")
        except OSError as e:
            log.error("I2C  write failed: %s", e)

    def stop(self):
        self.drive_speeds(0, 0)

    def cleanup(self):
        self.stop()
        if self._fd is not None:
            try:
                os.close(self._fd)
                log.info("I2C  fd closed")
            except OSError:
                pass
