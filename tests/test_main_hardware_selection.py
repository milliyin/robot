import argparse

import pytest

import main


class FakeArduino:
    def __init__(self, port):
        self.port = port
        self.connected = True
        self.error = None


class FakeJetson:
    def __init__(self, address):
        self.address = address
        self.connected = True
        self.error = None


def _args(**overrides):
    values = {
        "hardware": False,
        "jetson_pca9685": False,
        "port": "COM6",
        "i2c_address": 0x40,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def test_create_hardware_backend_keeps_arduino_path(monkeypatch):
    monkeypatch.setattr(main, "ArduinoSerial", FakeArduino)

    backend = main.create_hardware_backend(_args(hardware=True, port="COM7"))

    assert isinstance(backend, FakeArduino)
    assert backend.port == "COM7"


def test_create_hardware_backend_adds_jetson_path(monkeypatch):
    monkeypatch.setattr(main, "JetsonPCA9685", FakeJetson)

    backend = main.create_hardware_backend(_args(jetson_pca9685=True, i2c_address=0x41))

    assert isinstance(backend, FakeJetson)
    assert backend.address == 0x41


def test_create_hardware_backend_rejects_two_hardware_paths():
    with pytest.raises(ValueError, match="either --hardware"):
        main.create_hardware_backend(_args(hardware=True, jetson_pca9685=True))
