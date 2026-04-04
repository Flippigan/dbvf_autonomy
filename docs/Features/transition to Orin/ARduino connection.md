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

### Previous: PCA9685 Setup (removed)

Previously used an Adafruit PCA9685 over I2C:
- SDA -> Arduino A4
- SCL -> Arduino A5
- PCA9685 I2C address: 0x40

This was removed to simplify wiring — a single servo does not need a 16-channel driver.
