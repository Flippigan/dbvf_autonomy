# RC Direct Drop Mechanism — Design Spec

**Date:** 2026-04-05
**Status:** Approved

## Purpose

Add a manual payload drop capability controlled directly by RC transmitter switches, bypassing the Jetson/ROS2 stack entirely. The RC receiver PWM output wires directly into Arduino input pins. The Arduino reads the RC signal and drives a continuous rotation servo, stopping when a limit switch (micro switch) signals that one payload has been dispensed.

This coexists with the existing serial command protocol (`S0:<pwm>\n`) so the autonomous mission path remains fully functional.

## Hardware

### Continuous Rotation Servo

Already on pin D10 via the `Servo` library. For continuous rotation:
- ~1500μs = stop
- <1500μs = spin direction B (CCW)
- \>1500μs = spin direction A (CW)

### Pin Assignment

| Function | Pin | Config | Notes |
|----------|-----|--------|-------|
| Servo output (existing) | D10 | `Servo` library PWM | Continuous rotation servo |
| RC CH12 input | D5 | `INPUT` | RC receiver PWM signal — spin direction A |
| RC CH13 input | D6 | `INPUT` | RC receiver PWM signal — spin direction B |
| Limit switch | D2 | `INPUT_PULLUP` | COM→GND, NO→D2. HIGH=open, LOW=triggered |

### Wiring

```
RC Receiver CH12 ──signal──→ Arduino D5
RC Receiver CH13 ──signal──→ Arduino D6
(RC receiver GND shared with Arduino GND via flight controller power system)

Limit Switch COM  ──────────→ Arduino GND
Limit Switch NO   ──────────→ Arduino D2  (internal pullup → HIGH when open)
```

The limit switch uses the Arduino's internal pullup resistor. When the switch is not triggered (open), D2 reads HIGH. When the mechanism trips the switch (closed), D2 reads LOW.

## Firmware Logic

### Loop Structure

```
loop():
  1. Handle serial commands (existing — unchanged)
  2. Read RC CH12 via pulseIn(D5, HIGH, 25000)
  3. Read RC CH13 via pulseIn(D6, HIGH, 25000)
  4. Read limit switch via digitalRead(D2)
  5. Apply priority logic to servo
```

### Priority (highest to lowest)

1. **Limit switch triggered (D2 LOW)** → servo stops (1500μs), regardless of RC state
2. **RC CH12 high (>1700μs)** → servo spins direction A (CW_PWM)
3. **RC CH13 high (>1700μs)** → servo spins direction B (CCW_PWM)
4. **Both RC low / no signal** → no RC override, serial commands control servo as before

### Behavior Details

- **RC switch high + limit open:** Servo spins in commanded direction.
- **Limit triggers mid-spin:** Immediate stop. Servo stays at 1500μs as long as limit is held.
- **RC switch returns low:** RC override ends. Servo holds at whatever it was last set to. Serial commands resume full control.
- **Both RC switches high simultaneously:** CH12 wins (checked first). Not a realistic scenario with separate switches.
- **Serial command while RC is active:** RC override takes priority. Serial command is processed (ACK sent) but servo position will be overridden on next loop iteration by the RC logic. When RC switches return low, the last serial command's PWM would have already been applied to the servo.
- **No RC receiver connected:** `pulseIn()` times out (returns 0), both channels read as "low", serial-only mode works identically to the original firmware.

### pulseIn() Behavior

`pulseIn(pin, HIGH, 25000)` blocks for up to 25ms waiting for a HIGH pulse. Returns the pulse width in microseconds, or 0 on timeout. Two channels = up to 50ms blocking per loop in the worst case (no RC signal). This is acceptable because:
- Serial buffer is hardware-buffered (64 bytes on ATmega328P), commands won't be lost
- 50ms worst-case latency on serial response is fine for the autonomous use case
- When RC signal is present, `pulseIn()` returns in ~1-2ms per channel

## Tunable Constants

Defined at the top of the firmware for easy adjustment:

```cpp
// ── Pin assignments ──
static const int SERVO_PIN      = 10;
static const int RC_CH12_PIN    = 5;
static const int RC_CH13_PIN    = 6;
static const int LIMIT_SW_PIN   = 2;

// ── Servo PWM values (continuous rotation) ──
static const int CW_PWM         = 1700;   // Direction A speed
static const int CCW_PWM        = 1300;   // Direction B speed
static const int STOP_PWM       = 1500;   // Stop

// ── RC thresholds ──
static const int RC_THRESHOLD   = 1700;   // PWM above this = "switch high"
static const unsigned long RC_TIMEOUT_US = 25000;  // pulseIn timeout
```

## What Does Not Change

- Serial protocol: `S<channel>:<pwm_us>\n` → `OK\n` / `ERR:<reason>\n` — identical
- `arduino_interface_node.py` — no modifications
- All ROS2 nodes — no modifications
- Autonomous mission servo path — fully preserved
- Arduino USB serial connection to Jetson — unchanged

## File Modified

- `src/dbvf_autonomy/arduino/payload_servo_controller/payload_servo_controller.ino` — add RC input reading, limit switch reading, and priority-based servo control alongside existing serial command handler
