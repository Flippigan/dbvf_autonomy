// payload_servo_controller.ino
// Serial-to-Servo PWM bridge for payload servo mechanism.
// Protocol: S<channel>:<pwm_us>\n -> OK\n or ERR:<reason>\n
// Identical to arduino_interface_node protocol.
//
// Also reads RC receiver channels for manual payload drop control.
// RC switch above threshold = move servo, below = stop.
// Priority: RC > serial.
//
// Hardware: continuous rotation servo on pin D10 (channel 0).
// RC PWM read via pin-change interrupts (non-blocking).

#include <Servo.h>

// ── Baud rate ──
static const unsigned long BAUD_RATE = 115200;

// ── Pin assignments ──
static const int SERVO_PIN      = 10;
static const int RC_CH11_PIN    = 5;   // RC receiver CH11 — spin CW
static const int RC_CH12_PIN    = 6;   // RC receiver CH12 — spin CCW

// ── Servo PWM values (continuous rotation) ──
static const int CW_PWM         = 2500;  // Direction A max speed
static const int CCW_PWM        = 0;     // Direction B max speed
static const int STOP_PWM       = 1500;  // Stop

// ── RC thresholds ──
static const int RC_THRESHOLD   = 1200;  // PWM above this = "switch high"
static const unsigned long RC_STALE_US = 100000;  // no pulse for 100ms = no signal

Servo payloadServo;
String inputBuffer = "";

// ── Interrupt-based RC reading ──
volatile unsigned long ch11_rise = 0;
volatile unsigned long ch11_pw   = 0;
volatile unsigned long ch12_rise = 0;
volatile unsigned long ch12_pw   = 0;
volatile unsigned long ch11_last_update = 0;
volatile unsigned long ch12_last_update = 0;
volatile uint8_t prev_port_state = 0;

ISR(PCINT2_vect) {
  unsigned long now = micros();
  uint8_t cur = PIND;
  uint8_t changed = cur ^ prev_port_state;
  prev_port_state = cur;

  // CH11 on D5 (bit 5) — only process if this pin actually changed
  if (changed & (1 << 5)) {
    if (cur & (1 << 5)) {
      ch11_rise = now;
    } else if (ch11_rise > 0) {
      ch11_pw = now - ch11_rise;
      ch11_last_update = now;
      ch11_rise = 0;
    }
  }
  // CH12 on D6 (bit 6) — only process if this pin actually changed
  if (changed & (1 << 6)) {
    if (cur & (1 << 6)) {
      ch12_rise = now;
    } else if (ch12_rise > 0) {
      ch12_pw = now - ch12_rise;
      ch12_last_update = now;
      ch12_rise = 0;
    }
  }
}

void setup() {
  Serial.begin(BAUD_RATE);
  payloadServo.attach(SERVO_PIN);
  pinMode(RC_CH11_PIN, INPUT_PULLUP);
  pinMode(RC_CH12_PIN, INPUT_PULLUP);

  // Enable pin-change interrupts for D5 (PCINT21) and D6 (PCINT22)
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT21) | (1 << PCINT22);

  delay(10);
}

void loop() {
  // 1. Handle serial commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handleCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // 2. Read RC values (captured by interrupt)
  noInterrupts();
  unsigned long pw11 = ch11_pw;
  unsigned long pw12 = ch12_pw;
  unsigned long last11 = ch11_last_update;
  unsigned long last12 = ch12_last_update;
  interrupts();

  // Mark stale channels as 0
  unsigned long now = micros();
  if (now - last11 > RC_STALE_US) pw11 = 0;
  if (now - last12 > RC_STALE_US) pw12 = 0;

  // DEBUG: print RC readings
  Serial.print("RC11=");
  Serial.print(pw11);
  Serial.print(" RC12=");
  Serial.println(pw12);

  // 3. RC control: above threshold = move, below = stop
  if (pw11 > RC_THRESHOLD) {
    payloadServo.writeMicroseconds(CCW_PWM);
  } else if (pw12 > RC_THRESHOLD) {
    payloadServo.writeMicroseconds(CW_PWM);
  } else if (pw11 > 0 || pw12 > 0) {
    payloadServo.writeMicroseconds(STOP_PWM);
  }
  // else: no RC signal — serial commands retain control

  delay(20);  // ~50Hz loop rate
}

void handleCommand(const String& cmd) {
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
