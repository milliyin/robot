import time

from hardware.arduino_serial import ArduinoSerial


class FakeSerial:
    def __init__(self):
        self.writes = []
        self.closed = False

    def write(self, data):
        self.writes.append(data)

    def close(self):
        self.closed = True


def test_send_pwm_writes_newline_terminated_six_value_packet():
    fake = FakeSerial()
    bridge = ArduinoSerial(serial_obj=fake, max_hz=1000)

    assert bridge.send_pwm([284, 200, 550, 150, 190, 250]) is True

    assert fake.writes == [b"284,200,550,150,190,250\n"]


def test_send_pwm_clamps_each_channel_to_safe_range():
    fake = FakeSerial()
    bridge = ArduinoSerial(serial_obj=fake, max_hz=1000)

    bridge.send_pwm([-999, 999, 999, -999, 999, -999])

    assert fake.writes[-1] == b"90,420,550,75,555,250\n"


def test_send_pwm_is_rate_limited():
    fake = FakeSerial()
    bridge = ArduinoSerial(serial_obj=fake, max_hz=2)

    assert bridge.send_pwm([284, 200, 550, 150, 190, 250]) is True
    assert bridge.send_pwm([284, 200, 550, 150, 190, 250]) is False
    assert len(fake.writes) == 1

    time.sleep(0.51)
    assert bridge.send_pwm([284, 200, 550, 150, 190, 250]) is True
    assert len(fake.writes) == 2


def test_missing_serial_port_leaves_backend_disconnected():
    bridge = ArduinoSerial(port="COM_DOES_NOT_EXIST", serial_factory=lambda **kwargs: (_ for _ in ()).throw(OSError("missing")))

    assert bridge.connected is False
    assert bridge.send_pwm([284, 200, 550, 150, 190, 250]) is False
