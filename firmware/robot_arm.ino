#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const uint8_t CHANNELS = 6;
const uint16_t SAFE_MIN[CHANNELS] = {90, 75, 250, 75, 75, 250};
const uint16_t SAFE_MAX[CHANNELS] = {540, 420, 550, 470, 555, 400};
const uint16_t STARTUP_PWM[CHANNELS] = {284, 200, 550, 150, 190, 250};

String line;

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  applyTargets(STARTUP_PWM);
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      uint16_t targets[CHANNELS];
      if (parsePacket(line, targets)) {
        applyTargets(targets);
        Serial.println("OK");
      } else {
        Serial.println("ERR");
      }
      line = "";
    } else if (c != '\r') {
      line += c;
      if (line.length() > 48) {
        line = "";
      }
    }
  }
}

bool parsePacket(const String &line, uint16_t targets[CHANNELS]) {
  int start = 0;
  uint8_t field = 0;

  while (field < CHANNELS) {
    int comma = line.indexOf(',', start);
    int end = comma == -1 ? line.length() : comma;
    if (end <= start) {
      return false;
    }

    long value = 0;
    for (int i = start; i < end; i++) {
      char c = line.charAt(i);
      if (c < '0' || c > '9') {
        return false;
      }
      value = value * 10 + (c - '0');
      if (value > 1000) {
        return false;
      }
    }

    targets[field] = clampPulse(field, value);
    field++;
    if (comma == -1) {
      start = line.length();
      break;
    }
    start = comma + 1;
  }

  return field == CHANNELS && start == line.length();
}

uint16_t clampPulse(uint8_t channel, long value) {
  if (value < SAFE_MIN[channel]) {
    return SAFE_MIN[channel];
  }
  if (value > SAFE_MAX[channel]) {
    return SAFE_MAX[channel];
  }
  return (uint16_t)value;
}

void applyTargets(const uint16_t targets[CHANNELS]) {
  for (uint8_t channel = 0; channel < CHANNELS; channel++) {
    pwm.setPWM(channel, 0, targets[channel]);
  }
}
