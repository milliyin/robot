from pathlib import Path


def test_firmware_contains_exact_safe_ranges_and_startup_values():
    text = Path("firmware/robot_arm.ino").read_text(encoding="utf-8")
    compact = " ".join(text.split())
    assert "SAFE_MIN[CHANNELS] = {90, 75, 250, 75, 75, 250}" in compact
    assert "SAFE_MAX[CHANNELS] = {540, 420, 550, 470, 555, 400}" in compact
    assert "STARTUP_PWM[CHANNELS] = {284, 200, 550, 150, 190, 250}" in compact


def test_firmware_requires_exactly_six_fields_before_applying_packet():
    text = Path("firmware/robot_arm.ino").read_text(encoding="utf-8")
    assert "parsePacket(line, targets)" in text
    assert "return field == CHANNELS && start == line.length()" in text
    assert "applyTargets(targets)" in text
