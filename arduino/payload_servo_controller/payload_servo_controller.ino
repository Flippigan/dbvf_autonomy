// payload_servo_controller.ino
// Serial-to-Servo PWM bridge for payload servo mechanism.
// Protocol: S<channel>:<pwm_us>\n -> OK\n or ERR:<reason>\n
// Identical to arduino_interface_node protocol.
//
// Also reads RC receiver channels for manual payload drop control,
// with a limit switch for immediate stop. Priority: limit > RC > serial.
//
// Hardware: continuous rotation servo on pin D10 (channel 0).

#include <Servo.h>

// ── Baud rate ──
static const unsigned long BAUD_RATE = 115200;

// ── Pin assignments ──
static const int SERVO_PIN      = 10;
static const int RC_CH12_PIN    = 5;   // RC receiver CH12 — spin direction A
static const int RC_CH13_PIN    = 6;   // RC receiver CH13 — spin direction B
static const int LIMIT_SW_PIN   = 2;   // Micro limit switch (NO→D2, COM→GND)

// ── Servo PWM values (continuous rotation) ──
static const int CW_PWM         = 1700;  // Direction A speed
static const int CCW_PWM        = 1300;  // Direction B speed
static const int STOP_PWM       = 1500;  // Stop

// ── RC thresholds ──
static const int RC_THRESHOLD   = 1700;  // PWM above this = "switch high"
static const unsigned long RC_TIMEOUT_US = 25000;  // pulseIn timeout (μs)

Servo payloadServo;

String inputBuffer = "";

void setup() {
  Serial.begin(BAUD_RATE);
  payloadServo.attach(SERVO_PIN);
  pinMode(RC_CH12_PIN, INPUT);
  pinMode(RC_CH13_PIN, INPUT);
  pinMode(LIMIT_SW_PIN, INPUT_PULLUP);
  delay(10);
}

void loop() {
  // 1. Handle serial commands (existing — unchanged)
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // 2. Read RC channels via pulseIn (blocks up to RC_TIMEOUT_US each)
  unsigned long ch12_pw = pulseIn(RC_CH12_PIN, HIGH, RC_TIMEOUT_US);
  unsigned long ch13_pw = pulseIn(RC_CH13_PIN, HIGH, RC_TIMEOUT_US);

  // 3. Read limit switch (LOW = triggered due to INPUT_PULLUP)
  bool limitTriggered = (digitalRead(LIMIT_SW_PIN) == LOW);

  // 4. Priority: limit switch > RC CH12 > RC CH13 > serial (no override)
  if (limitTriggered) {
    payloadServo.writeMicroseconds(STOP_PWM);
  } else if (ch12_pw > RC_THRESHOLD) {
    payloadServo.writeMicroseconds(CW_PWM);
  } else if (ch13_pw > RC_THRESHOLD) {
    payloadServo.writeMicroseconds(CCW_PWM);
  }
  // else: no RC override — serial commands retain control
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
