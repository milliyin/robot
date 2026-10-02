from __future__ import annotations

import time
from typing import Callable


SAFE_PWM_RANGES = (
    (90, 540),
    (75, 420),
    (250, 550),
    (75, 470),
    (75, 555),
    (250, 400),
)


class ArduinoSerial:
    def __init__(
        self,
        port: str | None = None,
        baud: int = 115200,
        max_hz: float = 30,
        serial_obj=None,
        serial_factory: Callable | None = None,
    ):
        self.max_hz = float(max_hz)
        self.min_interval = 1.0 / self.max_hz if self.max_hz > 0 else 0.0
        self._last_send = 0.0
        self.serial = serial_obj
        self.connected = serial_obj is not None
        self.error: Exception | None = None

        if self.serial is None and port:
            try:
                if serial_factory is None:
                    import serial

                    serial_factory = serial.Serial
                self.serial = serial_factory(port=port, baudrate=baud, timeout=0.05)
                self.connected = True
            except Exception as exc:  # noqa: BLE001 - connection failures must not kill sim.
                self.error = exc
                self.connected = False

    def send_pwm(self, values: list[int]) -> bool:
        if not self.connected or self.serial is None:
            return False
        if len(values) != 6:
            raise ValueError("Expected exactly six PWM values")

        now = time.monotonic()
        if self._last_send and now - self._last_send < self.min_interval:
            return False

        clamped = [self._clamp_channel(i, value) for i, value in enumerate(values)]
        packet = ",".join(str(x) for x in clamped).encode("ascii") + b"\n"
        self.serial.write(packet)
        self._last_send = now
        return True

    def close(self) -> None:
        if self.serial is not None:
            self.serial.close()
        self.connected = False

    @staticmethod
    def _clamp_channel(index: int, value: int) -> int:
        lo, hi = SAFE_PWM_RANGES[index]
        return min(max(int(value), lo), hi)
