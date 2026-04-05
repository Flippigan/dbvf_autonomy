# RC Direct Drop Mechanism Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add RC-controlled continuous rotation servo driving and limit switch stop to the Arduino payload servo firmware, coexisting with the existing serial protocol.

**Architecture:** The existing `loop()` already handles serial commands. We add three new inputs — two RC PWM channels read via `pulseIn()` and one limit switch via `digitalRead()` — with a simple priority cascade (limit > RC > serial) evaluated every loop iteration after the serial handler. When no RC signal is present, the firmware behaves identically to the original.

**Tech Stack:** Arduino (ATmega328P), Servo library, `pulseIn()`, `digitalRead()` with `INPUT_PULLUP`

**Design Spec:** `docs/superpowers/specs/2026-04-05-rc-direct-drop-mechanism-design.md`

---

### Task 1: Add tunable constants and pin initialization

**Files:**
- Modify: `arduino/payload_servo_controller/payload_servo_controller.ino:1-21`

This task adds all configurable constants at the top of the file and configures the three new input pins in `setup()`. No behavioral change yet — the loop is unchanged.

- [ ] **Step 1: Replace the constants block at the top of the file**

Replace lines 9–12 (the existing `BAUD_RATE` and `SERVO_PIN` constants) with the full constants block. The existing `BAUD_RATE` and `SERVO_PIN` move into the new block:

```cpp
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
```

- [ ] **Step 2: Add pin configuration in setup()**

Add three `pinMode()` calls in `setup()`, after `payloadServo.attach(SERVO_PIN)` and before `delay(10)`:

```cpp
void setup() {
  Serial.begin(BAUD_RATE);
  payloadServo.attach(SERVO_PIN);
  pinMode(RC_CH12_PIN, INPUT);
  pinMode(RC_CH13_PIN, INPUT);
  pinMode(LIMIT_SW_PIN, INPUT_PULLUP);
  delay(10);
}
```

- [ ] **Step 3: Verify compilation**

Open Arduino IDE, select board "Arduino Nano" with processor "ATmega328P (Old Bootloader)", and click Verify (compile only). Expected: compiles with no errors or warnings.

- [ ] **Step 4: Commit**

```bash
cd /home/finn/Documents/ardu_ws/src/dbvf_autonomy
git add arduino/payload_servo_controller/payload_servo_controller.ino
git commit -m "feat(arduino): add RC and limit switch constants and pin init

Add tunable constants for RC CH12/CH13 pins, limit switch pin, servo
PWM values, and RC threshold. Configure new input pins in setup().
No behavioral change yet — loop is unchanged."
```

---

### Task 2: Add RC reading and priority logic in loop()

**Files:**
- Modify: `arduino/payload_servo_controller/payload_servo_controller.ino:24-34` (the `loop()` function)

This is the core change. After the existing serial handler in `loop()`, add RC channel reads, limit switch read, and the priority cascade. The serial handler code inside the `while (Serial.available())` block is **not modified at all** — we only add new code after it.

- [ ] **Step 1: Add RC and limit switch logic after the serial handler**

The current `loop()` is:

```cpp
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
```

Replace it with:

```cpp
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
```

Key behaviors:
- **Limit switch triggered:** Servo stops at 1500μs regardless of RC state.
- **CH12 high:** Servo spins CW (1700μs). CH12 wins over CH13 if both high.
- **CH13 high:** Servo spins CCW (1300μs).
- **Both low / no signal:** `pulseIn()` returns 0 (< RC_THRESHOLD), falls through to `else` — no write to servo, serial commands retain full control.
- **Serial while RC active:** `handleCommand()` writes to servo, then RC logic immediately overwrites on the same loop iteration. When RC goes low, servo holds at whatever was last written.

- [ ] **Step 2: Verify compilation**

Open Arduino IDE, select board "Arduino Nano" with processor "ATmega328P (Old Bootloader)", and click Verify (compile only). Expected: compiles with no errors or warnings.

- [ ] **Step 3: Commit**

```bash
cd /home/finn/Documents/ardu_ws/src/dbvf_autonomy
git add arduino/payload_servo_controller/payload_servo_controller.ino
git commit -m "feat(arduino): add RC channel reading and limit switch priority logic

Read RC CH12/CH13 via pulseIn() and limit switch via digitalRead()
after the serial handler in loop(). Priority: limit switch (immediate
stop) > RC CH12 (CW) > RC CH13 (CCW) > serial commands (no override).
When no RC signal present, firmware behaves identically to original."
```

---

### Task 3: Upload and verify on hardware

**Files:**
- No file changes — hardware verification only

This task verifies the firmware on actual hardware. All four operating modes must be tested.

- [ ] **Step 1: Upload firmware to Arduino Nano**

In Arduino IDE:
- Board: "Arduino Nano"
- Processor: "ATmega328P (Old Bootloader)"
- Port: select the USB serial port (e.g., `/dev/ttyACM0`)
- Click Upload

Expected: upload succeeds, no errors.

- [ ] **Step 2: Verify serial protocol is unchanged (no RC receiver connected)**

Open Arduino IDE Serial Monitor at 115200 baud. Send these commands and verify responses:

| Send | Expected Response |
|------|-------------------|
| `S0:1500` | `OK` |
| `S0:1700` | `OK` |
| `S0:1300` | `OK` |
| `S1:1500` | `ERR:channel out of range` |
| `S0:25000` | `ERR:pwm out of range` |
| `hello` | `ERR:bad format` |

With no RC receiver connected, `pulseIn()` times out and returns 0 for both channels. The firmware should behave identically to the original — servo responds to serial commands only. Verify the servo physically moves when given different PWM values.

- [ ] **Step 3: Verify RC CH12 drives servo CW**

Connect RC receiver CH12 signal wire to Arduino D5. Power on transmitter, flip CH12 switch high. Expected: servo spins CW (direction A). Flip switch low. Expected: servo holds at last position (CW speed). Send `S0:1500` via serial to stop.

- [ ] **Step 4: Verify RC CH13 drives servo CCW**

Connect RC receiver CH13 signal wire to Arduino D6. Flip CH13 switch high. Expected: servo spins CCW (direction B). Flip switch low. Expected: servo holds. Send `S0:1500` via serial to stop.

- [ ] **Step 5: Verify limit switch stops servo immediately**

Connect limit switch (COM→GND, NO→D2). Start servo spinning via RC CH12. While spinning, press/trigger the limit switch. Expected: servo stops immediately (1500μs). Release limit switch. Expected: servo stays stopped (no RC channel is high, falls through to no-override). Flip RC switch again to resume spinning.

- [ ] **Step 6: Verify limit switch overrides serial commands**

Send `S0:1700` via serial (servo spins CW). Press limit switch. Expected: servo stops. The serial command was already acknowledged with `OK`, but the limit switch overrides the servo output.

---

## Complete Modified Firmware Reference

For reference, the complete firmware after all tasks should look like this:

```cpp
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
```
