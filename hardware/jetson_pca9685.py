from __future__ import annotations

import time
from typing import Callable

from hardware.arduino_serial import SAFE_PWM_RANGES


def pulse_to_duty_cycle(pulse: int) -> int:
    pulse = min(max(int(pulse), 0), 4095)
    return int(round(pulse * 65535 / 4095))


class JetsonPCA9685:
    def __init__(
        self,
        address: int = 0x40,
        frequency: int = 50,
        max_hz: float = 30,
        pca_obj=None,
        pca_factory: Callable | None = None,
    ):
        self.max_hz = float(max_hz)
        self.min_interval = 1.0 / self.max_hz if self.max_hz > 0 else 0.0
        self._last_send = 0.0
        self.pca = pca_obj
        self.connected = pca_obj is not None
        self.error: Exception | None = None

        if self.pca is None:
            try:
                if pca_factory is None:
                    import board
                    import busio
                    from adafruit_pca9685 import PCA9685

                    i2c = busio.I2C(board.SCL, board.SDA)
                    pca_factory = PCA9685
                    self.pca = pca_factory(i2c, address=address)
                else:
                    self.pca = pca_factory(address=address)
                self.connected = True
            except Exception as exc:  # noqa: BLE001 - missing Jetson/I2C libs must not kill sim.
                self.error = exc
                self.connected = False

        if self.connected and self.pca is not None:
            self.pca.frequency = frequency

    def send_pwm(self, values: list[int]) -> bool:
        if not self.connected or self.pca is None:
            return False
        if len(values) != 6:
            raise ValueError("Expected exactly six PWM values")

        now = time.monotonic()
        if self._last_send and now - self._last_send < self.min_interval:
            return False

        for channel, value in enumerate(values):
            pulse = self._clamp_channel(channel, value)
            self.pca.channels[channel].duty_cycle = pulse_to_duty_cycle(pulse)
        self._last_send = now
        return True

    def close(self) -> None:
        if self.pca is not None and hasattr(self.pca, "deinit"):
            self.pca.deinit()
        self.connected = False

    @staticmethod
    def _clamp_channel(index: int, value: int) -> int:
        lo, hi = SAFE_PWM_RANGES[index]
        return min(max(int(value), lo), hi)
