# RC Direct Drop Mechanism — Progress

## Status: Complete — RC servo control verified and committed

## What Was Done

### Firmware Changes (payload_servo_controller.ino)

RC-controlled servo with two channels:

| Input | Pin | Direction | Servo PWM |
|-------|-----|-----------|-----------|
| RC CH11 | D5 | CCW | 0us (max speed) |
| RC CH12 | D6 | CW | 2500us (max speed) |

**Logic:** RC switch above 1200us threshold = move servo at max speed, below = stop (1500us). When no RC signal present, serial commands retain control.

**Priority:** RC > serial commands.

### Wiring (Current Setup)

| Connection | Arduino Pin | Notes |
|------------|-------------|-------|
| Servo signal | D10 | Continuous rotation servo |
| RC receiver CH11 signal | D5 | CW direction (from Cube Orange SERVO11 output) |
| RC receiver CH12 signal | D6 | CCW direction (from Cube Orange SERVO12 output) |
| Power from Cube Orange | 5V + GND | Do not use VIN |

### Cube Orange Configuration Required

- `SERVO11_FUNCTION = 61` (RCPassThru for RC input 11)
- `SERVO12_FUNCTION = 62` (RCPassThru for RC input 12)

Without these, the Cube Orange outputs a fixed ~960us regardless of RC switch position.

### Pin-Change Interrupt RC Reading (2026-04-05)

Replaced blocking `pulseIn()` with pin-change interrupts (`PCINT2_vect` on PORTD). Two sequential `pulseIn` calls blocked ~25ms each and caused one channel to always miss its pulse. The ISR reads `PIND` directly and XORs with previous state to detect which pin actually changed — critical because PCINT fires for ANY pin change in the port group.

Stale readings (>100ms since last pulse) are treated as no signal, allowing serial commands to work when no RC receiver is connected.

### Serial \r Handling (2026-04-05)

Added `\r` stripping in the serial input loop so commands work regardless of Serial Monitor line ending setting.

### Arduino IDE Settings

- Board: **Arduino Nano**
- Processor: **ATmega328P (Old Bootloader)**
- Baud: 115200
- Port: /dev/ttyUSB0

## Verified

- [x] Serial protocol works (S0:1500, S0:2500, S0:500, error cases)
- [x] No spurious servo commands when RC receiver disconnected (INPUT_PULLUP + stale detection)
- [x] RC CH11 drives servo CCW (max speed 0us)
- [x] RC CH12 drives servo CW (max speed 2500us)
- [x] Both RC channels read simultaneously (pin-change interrupts)
- [x] Servo stops when RC switch returned to low position
- [ ] Full integration with RC receiver on drone (in-flight test)

## Bugs Fixed (2026-04-05)

| Bug | Root Cause | Fix |
|-----|-----------|-----|
| RC channels not activating servo | RC_THRESHOLD was 1700, CH12 high output was ~1495 | Lowered threshold to 1200 |
| Only one RC channel readable at a time | Two sequential `pulseIn()` calls blocked ~25ms each, second always missed | Replaced with PCINT2 pin-change interrupts |
| Wrong channel detected by interrupt | PCINT fires for any pin change in port; ISR processed pins that didn't change | Added XOR edge detection with `prev_port_state` |
| Floating RC pins causing erratic behavior | D5/D6 set to `INPUT` with no receiver | Changed to `INPUT_PULLUP` |
| Limit switch blocking RC commands | Limit latch checked before RC branches, RC could never clear it | Removed limit switch logic (not needed for competition) |

## Removed Features (2026-04-05)

- **Limit switch** (D2) — removed entirely, not needed for competition drop mechanism
- **FORCE/NOFORCE** serial commands — only existed for limit switch bypass

## Files Modified

- `arduino/payload_servo_controller/payload_servo_controller.ino` — RC interrupt reading, simplified logic, removed limit switch

## Design Docs

- Spec: `docs/superpowers/specs/2026-04-05-rc-direct-drop-mechanism-design.md`
- Plan: `docs/superpowers/plans/2026-04-05-rc-direct-drop-mechanism.md`
