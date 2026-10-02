import time

from hardware.jetson_pca9685 import JetsonPCA9685, pulse_to_duty_cycle


class FakeChannel:
    def __init__(self):
        self.duty_cycle = None


class FakePCA:
    def __init__(self):
        self.channels = [FakeChannel() for _ in range(16)]
        self.frequency = None
        self.deinitialized = False

    def deinit(self):
        self.deinitialized = True


def test_pulse_to_duty_cycle_scales_12_bit_pca_ticks_to_16_bit_duty():
    assert pulse_to_duty_cycle(0) == 0
    assert pulse_to_duty_cycle(4095) == 65535
    assert pulse_to_duty_cycle(250) == round(250 * 65535 / 4095)


def test_send_pwm_writes_clamped_six_channel_duty_cycles():
    fake = FakePCA()
    bridge = JetsonPCA9685(pca_obj=fake, max_hz=1000)

    assert bridge.send_pwm([-999, 999, 999, -999, 999, -999]) is True

    expected_pulses = [90, 420, 550, 75, 555, 250]
    assert [channel.duty_cycle for channel in fake.channels[:6]] == [
        pulse_to_duty_cycle(pulse) for pulse in expected_pulses
    ]


def test_send_pwm_is_rate_limited():
    fake = FakePCA()
    bridge = JetsonPCA9685(pca_obj=fake, max_hz=2)

    assert bridge.send_pwm([284, 200, 550, 150, 190, 250]) is True
    first = [channel.duty_cycle for channel in fake.channels[:6]]

    assert bridge.send_pwm([90, 75, 250, 75, 75, 250]) is False
    assert [channel.duty_cycle for channel in fake.channels[:6]] == first

    time.sleep(0.51)
    assert bridge.send_pwm([90, 75, 250, 75, 75, 250]) is True


def test_missing_i2c_backend_leaves_bridge_disconnected():
    def failing_factory(*args, **kwargs):
        raise OSError("no i2c")

    bridge = JetsonPCA9685(pca_factory=failing_factory)

    assert bridge.connected is False
    assert bridge.send_pwm([284, 200, 550, 150, 190, 250]) is False


def test_close_deinitializes_pca9685():
    fake = FakePCA()
    bridge = JetsonPCA9685(pca_obj=fake)

    bridge.close()

    assert fake.deinitialized is True
    assert bridge.connected is False
