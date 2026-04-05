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
static const int LIMIT_SW_PIN   = 2;   // Micro limit switch (NC→D2, COM→GND)

// ── Servo PWM values (continuous rotation) ──
static const int CW_PWM         = 1700;  // Direction A speed
static const int CCW_PWM        = 1300;  // Direction B speed
static const int STOP_PWM       = 1500;  // Stop

// ── RC thresholds ──
static const int RC_THRESHOLD   = 1700;  // PWM above this = "switch high"
static const unsigned long RC_TIMEOUT_US = 25000;  // pulseIn timeout (μs)

Servo payloadServo;

String inputBuffer = "";
bool forceOverride = false;  // serial "FORCE" disables limit switch
bool prevLimitPressed = false;  // edge detection for limit switch
bool limitLatched = false;       // true after rising edge, cleared by new command

void setup() {
  Serial.begin(BAUD_RATE);
  payloadServo.attach(SERVO_PIN);
  pinMode(RC_CH12_PIN, INPUT_PULLUP);
  pinMode(RC_CH13_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SW_PIN, INPUT_PULLUP);
  delay(10);
}

void loop() {
  // 1. Handle serial commands (existing — unchanged)
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;  // strip carriage return
    if (c == '\n') {
      if (inputBuffer == "FORCE") {
        forceOverride = true;
        Serial.println("OK:force on");
      } else if (inputBuffer == "NOFORCE") {
        forceOverride = false;
        Serial.println("OK:force off");
      } else {
        handleCommand(inputBuffer);
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // 2. Read RC channels via pulseIn (blocks up to RC_TIMEOUT_US each)
  unsigned long ch12_pw = pulseIn(RC_CH12_PIN, HIGH, RC_TIMEOUT_US);
  unsigned long ch13_pw = pulseIn(RC_CH13_PIN, HIGH, RC_TIMEOUT_US);

  // 3. Limit switch edge detection: stop once on contact, then allow
  //    new commands to move servo off the switch.
  bool limitPressed = (digitalRead(LIMIT_SW_PIN) == LOW);
  if (limitPressed && !prevLimitPressed) {
    // Rising edge: switch just depressed — stop and latch
    limitLatched = true;
    payloadServo.writeMicroseconds(STOP_PWM);
  }
  prevLimitPressed = limitPressed;

  // 4. Priority: latched limit > RC CH12 > RC CH13 > serial
  if (limitLatched && !forceOverride) {
    // Stay stopped until a new serial or RC command clears the latch
  } else if (ch12_pw > RC_THRESHOLD) {
    limitLatched = false;
    payloadServo.writeMicroseconds(CW_PWM);
  } else if (ch13_pw > RC_THRESHOLD) {
    limitLatched = false;
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

  limitLatched = false;  // new command clears limit latch
  payloadServo.writeMicroseconds(pwm_us);

  Serial.println("OK");
}
