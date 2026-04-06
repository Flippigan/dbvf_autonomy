# RC Direct Drop Mechanism — Progress

## Status: Hardware Testing In Progress

## What Was Done

### Firmware Changes (payload_servo_controller.ino)

Added three new inputs to the existing serial-controlled servo firmware:

| Input | Pin | Purpose |
|-------|-----|---------|
| RC CH11 | D5 | Spin servo CW (1700us) via RC switch |
| RC CH12 | D6 | Spin servo CCW (1300us) via RC switch |
| Limit switch | D2 | Stop servo (1500us) when depressed |

**Priority cascade:** latched limit switch > RC channels > serial commands

When no RC receiver is connected, firmware behaves identically to the original serial-only version.

### Wiring (Current Setup)

| Connection | Arduino Pin | Notes |
|------------|-------------|-------|
| Servo signal | D10 | Continuous rotation servo |
| RC receiver CH11 signal | D5 | CW direction |
| RC receiver CH12 signal | D6 | CCW direction |
| Limit switch COM | GND | |
| Limit switch | D2 | INPUT_PULLUP |
| Power from Cube Orange | 5V + GND | Do not use VIN |

### Limit Switch Behavior (2026-04-05 debugging)

Actual behavior with `INPUT_PULLUP` on D2:
- **Undepressed** (at rest): D2 reads HIGH — servo moves freely
- **Depressed**: D2 reads LOW — servo stops

Code uses `digitalRead(LIMIT_SW_PIN) == LOW` to detect the depressed state.

**Note:** Initial code assumed NC wiring (depressed = HIGH), but diagnostic testing confirmed the opposite polarity. The logic was inverted to match actual hardware readings.

### Edge-Triggered Limit Switch (2026-04-05)

The limit switch uses **edge detection + latch** instead of level-based stopping:
- **Rising edge** (undepressed → depressed): servo stops immediately, latch is set
- **While latched**: servo stays stopped — prevents drift
- **New serial or RC command**: clears the latch, allowing the servo to move off the switch

This prevents the servo from being permanently locked when the mechanism physically holds the switch depressed.

### RC Pin Mode Fix (2026-04-05)

RC input pins changed from `INPUT` to `INPUT_PULLUP` to prevent floating pin noise from generating spurious pulseIn readings when no RC receiver is connected.

### Serial \r Handling (2026-04-05)

Added `\r` stripping in the serial input loop so FORCE/NOFORCE commands work regardless of Serial Monitor line ending setting (NL, CR, or Both).

### Force Override (for positioning)

Serial commands `FORCE` and `NOFORCE` toggle a flag that bypasses the limit switch check. Used to reposition the servo past the switch during setup:

1. Send `FORCE` in Serial Monitor (115200 baud) — responds `OK:force on`
2. Send `S0:1700` or `S0:1300` to rotate freely
3. Send `NOFORCE` to re-enable the limit switch — responds `OK:force off`

### Arduino IDE Settings

- Board: **Arduino Nano**
- Processor: **ATmega328P (Old Bootloader)**
- Baud: 115200

## Verified

- [x] Serial protocol works (S0:1500, S0:1700, S0:1300, error cases)
- [x] Limit switch stops servo when depressed (edge-triggered + latch)
- [x] Servo can be commanded off switch after limit latch (new command clears latch)
- [x] FORCE/NOFORCE override allows positioning past limit switch
- [x] No spurious servo commands when RC receiver disconnected (INPUT_PULLUP fix)
- [ ] RC CH11 drives servo CW
- [ ] RC CH12 drives servo CCW
- [ ] Limit switch overrides RC channels
- [ ] Full integration with RC receiver on drone

## Bugs Fixed (2026-04-05)

| Bug | Root Cause | Fix |
|-----|-----------|-----|
| Servo won't rotate when switch undepressed | Limit switch polarity inverted — code checked `== HIGH` but undepressed reads HIGH | Changed to `== LOW` |
| Servo permanently locked after hitting switch | Level-based limit check ran every loop, overwriting commands | Edge detection + latch, cleared by new command |
| Floating RC pins causing erratic behavior | D5/D6 set to `INPUT` with no receiver | Changed to `INPUT_PULLUP` |
| FORCE command returns ERR:bad format | Serial Monitor sends `\r\n`, `\r` stayed in buffer | Added `\r` stripping |

## Files Modified

- `arduino/payload_servo_controller/payload_servo_controller.ino` — added RC input, limit switch, force override, edge detection, INPUT_PULLUP, \r strip

## Design Docs

- Spec: `docs/superpowers/specs/2026-04-05-rc-direct-drop-mechanism-design.md`
- Plan: `docs/superpowers/plans/2026-04-05-rc-direct-drop-mechanism.md`
