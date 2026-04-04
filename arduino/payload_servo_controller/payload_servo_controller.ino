// payload_servo_controller.ino
// Serial-to-Servo PWM bridge for payload servo mechanism.
// Protocol: S<channel>:<pwm_us>\n -> OK\n or ERR:<reason>\n
// Identical to arduino_interface_node protocol.
//
// Hardware: single servo on pin D10 (channel 0).
// Replaces previous PCA9685 I2C approach — same protocol, simpler wiring.

#include <Servo.h>

static const unsigned long BAUD_RATE = 115200;
static const int SERVO_PIN = 10;

Servo payloadServo;

String inputBuffer = "";

void setup() {
  Serial.begin(BAUD_RATE);
  payloadServo.attach(SERVO_PIN);
  delay(10);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

void handleCommand(const String& cmd) {
  // Expected format: S<channel>:<pwm_us>
  if (cmd.length() < 4 || cmd.charAt(0) != 'S') {
    Serial.println("ERR:bad format");
    return;
  }

  int colonIdx = cmd.indexOf(':');
  if (colonIdx < 0) {
    Serial.println("ERR:missing colon");
    return;
  }

  int channel = cmd.substring(1, colonIdx).toInt();
  int pwm_us = cmd.substring(colonIdx + 1).toInt();

  if (channel != 0) {
    Serial.println("ERR:channel out of range");
    return;
  }

  if (pwm_us < 0 || pwm_us > 20000) {
    Serial.println("ERR:pwm out of range");
    return;
  }

  payloadServo.writeMicroseconds(pwm_us);

  Serial.println("OK");
}
