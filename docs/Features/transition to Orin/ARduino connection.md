# Arduino Nano — Payload Servo Connection

## Current: Direct Servo from Arduino Nano (no PCA9685)

The Adafruit PCA9685 16-Channel Servo Driver has been removed. The payload servo is driven directly from the Arduino Nano.

### Wiring

| Servo Wire | Arduino Nano Pin |
|------------|------------------|
| Signal (orange/white) | **D9** (PWM-capable) |
| VCC (red) | External 5-6V supply (NOT the Arduino 5V pin — servo draws too much current) |
| GND (brown/black) | GND (shared with external supply) |

### PWM-capable pins on Arduino Nano

D3, D5, D6, D9, D10, D11 (marked with `~` on the board). D9 is used by default.

### Software

Uses the Arduino `Servo` library instead of the Adafruit PWM Servo Driver library. The serial protocol (`S<channel>:<pwm_us>\n` -> `OK\n` / `ERR:<reason>\n`) remains unchanged — the `arduino_interface_node` requires no modifications.

### Power Supply Requirements

| Component | Typical Draw |
|-----------|-------------|
| Arduino Nano | ~50 mA |
| Standard hobby servo (idle) | ~10-50 mA |
| Standard hobby servo (moving, loaded) | ~200-800 mA |
| Standard hobby servo (stall) | ~1-2 A |

**Recommended PSU output: 2A** for a standard 9g/SG90-class servo (Nano + servo under load + stall headroom). For higher-torque metal-gear servos (e.g. MG996R), use **3A** (stall up to 2.5A).

Check your servo's datasheet for stall current and add ~0.5A margin.

### Ground Sharing

The external PSU ground must be shared between the Arduino Nano and the servo. Use a Y-split (solder joint, terminal block, or breadboard ground rail) so both devices reference the same ground potential. Without this, the D9 PWM signal has no common voltage reference and the servo won't respond reliably.

```
                    ┌─────────────────────┐
                    │    External PSU      │
                    │    (5-6V supply)     │
                    │  [+] ──────────┐    │
                    │  [-] ──────┐   │    │
                    └────────────┼───┼────┘
                                 │   │
                                 │   │  VCC (red)
                                 │   └──────────────┐
                                 │                   │
                         GND from PSU                │
                                 │                   │
                                 ├───────────┐       │
                                 │    Y-split │      │
                                 │    (ground)│      │
                                 ▼            ▼      ▼
                          ┌──────────┐ ┌──────────────────┐
                          │ Arduino  │ │  Payload Servo    │
                          │  Nano    │ │                   │
                          │ GND pin  │ │ GND (brown/black) │
                          │          │ │ VCC (red)         │
                          │ D9  pin ─┼─▶ Signal (orange)  │
                          │ (PWM ~)  │ └──────────────────┘
                          └──────────┘
```

### Previous: PCA9685 Setup (removed)

Previously used an Adafruit PCA9685 over I2C:
- SDA -> Arduino A4
- SCL -> Arduino A5
- PCA9685 I2C address: 0x40

This was removed to simplify wiring — a single servo does not need a 16-channel driver.
