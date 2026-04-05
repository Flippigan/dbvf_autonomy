# RC Direct Drop Mechanism — Progress

## Status: Hardware Testing In Progress

## What Was Done

### Firmware Changes (payload_servo_controller.ino)

Added three new inputs to the existing serial-controlled servo firmware:

| Input | Pin | Purpose |
|-------|-----|---------|
| RC CH12 | D5 | Spin servo CW (1700us) via RC switch |
| RC CH13 | D6 | Spin servo CCW (1300us) via RC switch |
| Limit switch | D2 | Stop servo (1500us) when depressed |

**Priority cascade:** limit switch > RC channels > serial commands

When no RC receiver is connected, firmware behaves identically to the original serial-only version.

### Wiring (Current Setup)

| Connection | Arduino Pin | Notes |
|------------|-------------|-------|
| Servo signal | D10 | Continuous rotation servo |
| RC receiver CH12 signal | D5 | CW direction |
| RC receiver CH13 signal | D6 | CCW direction |
| Limit switch COM | GND | |
| Limit switch NC | D2 | Using NC terminal (not NO) |
| Limit switch NO | — | Disconnected |
| Power from Cube Orange | 5V + GND | Do not use VIN |

### Limit Switch Wiring Discovery

The limit switch is wired using the **NC (normally closed)** terminal, not NO. With `INPUT_PULLUP` on D2:
- **At rest** (NC closed): D2 pulled to GND via switch, reads LOW — servo moves freely
- **Depressed** (NC opens): internal pull-up pulls D2 HIGH — servo stops

Code uses `digitalRead(LIMIT_SW_PIN) == HIGH` to detect the depressed state.

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
- [x] Limit switch stops servo when depressed
- [x] FORCE/NOFORCE override allows positioning past limit switch
- [ ] RC CH12 drives servo CW
- [ ] RC CH13 drives servo CCW
- [ ] Limit switch overrides RC channels
- [ ] Full integration with RC receiver on drone

## Files Modified

- `arduino/payload_servo_controller/payload_servo_controller.ino` — added RC input, limit switch, force override

## Design Docs

- Spec: `docs/superpowers/specs/2026-04-05-rc-direct-drop-mechanism-design.md`
- Plan: `docs/superpowers/plans/2026-04-05-rc-direct-drop-mechanism.md`
