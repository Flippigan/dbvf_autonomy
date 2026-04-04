# Final Positioning During Landing — Issues

## ISS-013: 4-phase precision landing final approach

**Date:** 2026-04-02
**Status:** IMPLEMENTED (2026-04-03)
**Implementation plan:** `docs/superpowers/plans/2026-04-02-precision-landing-4phase.md`

### Problem (Original)

The precision landing FSM had no visibility or control over the sequencing between tag detection, position stabilization, yaw alignment, and lateral offset. DESCEND_COARSE transitioned directly to DESCEND_OFFSET, and there was no rotational alignment phase.

### Solution Implemented

Restructured the final approach into a 4-phase sequence with two new FSM states and a rename:

**Old flow:** DESCEND_COARSE → DESCEND_OFFSET → DESCEND_FINAL
**New flow:** DESCEND_COARSE → HOLD_ABOVE_TAG → ALIGN_YAW → OFFSET_LATERAL → DESCEND_FINAL

| Phase | State | vz | Purpose |
|-------|-------|----|---------|
| 1 | HOLD_ABOVE_TAG | 0.0 | Hold position above secondary tag, wait for PID to stabilize (0.5s) |
| 2 | ALIGN_YAW | 0.0 | Rotate to target yaw relative to tag (P controller on tag_yaw). Auto-skips if tag_yaw unavailable (ISS-015 not yet implemented) |
| 3 | OFFSET_LATERAL | 0.0 | Move to lateral offset position (offset_forward/offset_right) |
| 4 | DESCEND_FINAL | final_descent_rate | Final descent while maintaining offset |

All new states track tag-lost timeout (4s → SEARCH) independently.

### Changes Made

| File | Change |
|------|--------|
| `src/dbvf_msgs/srv/StartPrecisionLanding.srv` | Added `float64 target_yaw 0.0` |
| `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py` | Added `wrap_angle()`, HOLD_ABOVE_TAG + ALIGN_YAW states/handlers, renamed DESCEND_OFFSET → OFFSET_LATERAL, P controller yaw rate command, `_call_guided_velocity` accepts `yaw_rate=` kwarg, 6 new ROS params |
| `src/dbvf_autonomy/test/test_state_machine.py` | 20 new tests (53 total): wrap_angle (7), HOLD_ABOVE_TAG (5), ALIGN_YAW (5), preferred tag for new states (3) |
| `src/dbvf_autonomy/dbvf_autonomy/mission_state_machine.py` | Updated `'DESCEND_OFFSET'` → `'OFFSET_LATERAL'` string check for WA reload |
| `src/dbvf_autonomy/test/test_mission_state_machine.py` | Updated string references |
| `src/dbvf_autonomy/dbvf_autonomy/mission_sequencer_node.py` | Passes `target_yaw` through to `start_precision_landing` |
| `src/dbvf_autonomy/config/sim_params.yaml` | 6 new params: hold_position_tolerance, hold_stabilize_time, yaw_alignment_tolerance, yaw_alignment_hold_time, yaw_kp, max_yaw_rate |
| `src/dbvf_autonomy/config/hardware_params.yaml` | Same 6 params |
| `src/dbvf_autonomy/config/mission_params.yaml` | Added `wa_target_yaw: 0.0` |

### Dependency on ISS-015

ALIGN_YAW uses `getattr(target, 'tag_yaw', None)` — gracefully auto-skips to OFFSET_LATERAL when `tag_yaw` field doesn't exist on `LandingTargetPose`. Once ISS-015 adds the `tag_yaw` field, yaw alignment will activate automatically.

### Verification

235 tests, 0 failures. 10 commits on `feat/wa-reload-mechanism` (dbvf_autonomy) + 1 on `feat/mission-sequencer` (dbvf_msgs).

---

## ISS-014: Continuous yaw spin after secondary tag detection (yaw hold regression)

**Date:** 2026-04-02
**Status:** FIXED
**Introduced by:** Yaw hold implementation (added `use_yaw` + `yaw` fields to `SendGuidedVelocity`)
**Fix commit:** `b044df9` — `fix(ISS-014): use yaw=0.0 for body-frame heading hold`

### Symptom

After detecting the smaller AprilTag (ID 2), the drone begins spinning continuously to the right. It does not stop spinning and eventually collides with the ground while rotating.

### Root Cause

Frame mismatch between yaw value and coordinate frame. `precision_landing_node.py:661` sent `req.yaw = math.radians(vs.heading)` (absolute heading in radians) through `MAV_FRAME_BODY_NED` (frame 8). ArduPilot (`GCS_MAVLink_Copter.cpp:1372`) sets `yaw_relative = true` for BODY_NED frames, interpreting yaw as a rotation relative to current heading. Sending e.g. 1.57 rad (90 deg heading) every tick commanded +90 deg relative rotation repeatedly — positive feedback loop — accelerating spin.

### Fix

Changed `req.yaw = math.radians(vs.heading)` to `req.yaw = 0.0` at `precision_landing_node.py:661`. In `MAV_FRAME_BODY_NED`, `yaw = 0.0` means "rotate 0 degrees from current heading" = hold heading.

Regression test added: `test_state_machine.py::test_body_frame_yaw_hold_uses_zero`

### Key Lesson

In MAVLink `SET_POSITION_TARGET_LOCAL_NED`:
- `MAV_FRAME_BODY_NED` / `MAV_FRAME_BODY_OFFSET_NED`: yaw is **body-relative**
- `MAV_FRAME_LOCAL_NED` / `MAV_FRAME_LOCAL_OFFSET_NED`: yaw is **absolute** (NED North=0)

### Files Changed

- `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py:661`
- `src/dbvf_autonomy/test/test_state_machine.py` (regression test)

Full trace: `docs/superpowers/Log/ISS-014-body-frame-yaw-spin.md`

---

## ISS-015: No tag-relative yaw — drone cannot orient itself relative to the landing pad

**Date:** 2026-04-02
**Status:** Open — enhancement opportunity

### Problem

The current system can position the drone in x, y, z relative to the AprilTag, but has **no ability to control orientation relative to the tag**. The yaw hold (`yaw=0.0`) just freezes whatever heading the drone arrived with. If the drone needs to land with a specific orientation (e.g., pickup mechanism aligned to the pad), there is no way to command that.

### What the system already has

The homography from apriltag_ros contains full 6DOF information (position + rotation), but only the **translation** is currently extracted:

- **Position (x, y, z):** `estimate_tag_position()` in `tag_detector_adapter_node.py:82-118` decomposes `K^-1 * H` into rotation columns and a translation column. It extracts translation only. This is published as `position_x/y/z` in `LandingTargetPose` and used by the precision landing PID loop.

- **Rotation (yaw):** The same decomposition computes the first two rotation matrix columns (`m00,m10,m20` and `m01,m11,m21` at lines 98-104) but discards them. The tag-relative yaw could be extracted as:
  ```python
  yaw_rad = math.atan2(m10, m00)  # rotation of drone around camera optical axis relative to tag
  ```

### What this would enable

1. **Tag-relative yaw extraction** — know the drone's heading relative to the tag's printed orientation, updated at camera framerate (no compass dependency)
2. **Active yaw alignment** — close the yaw loop visually by sending `yaw = -tag_relative_yaw` each tick via `SendGuidedVelocity`, actively aligning the drone to a desired orientation relative to the tag
3. **Mechanism alignment** — if the pickup mechanism must face a specific direction relative to the pad, the tag defines the reference frame

### Current yaw command capabilities

The `SendGuidedVelocity` service already supports yaw control in `MAV_FRAME_BODY_NED`:

| Field | Meaning in BODY_NED | Example |
|-------|---------------------|---------|
| `yaw` + `use_yaw=True` | Body-relative angle — rotate N radians from current heading | `yaw=0.523` = rotate 30 deg CW |
| `yaw_rate` + `use_yaw_rate=True` | Continuous rotation rate (rad/s) | `yaw_rate=0.175` = 10 deg/s |
| `yaw=0.0` + `use_yaw=True` | Hold current heading (ISS-014 fix) | Current behavior |

Positive yaw = clockwise, negative = counter-clockwise.

### Implementation path

1. **Extract yaw from homography** — add `tag_yaw` field to `LandingTargetPose` message, compute it in `estimate_tag_position()` or a new `estimate_tag_yaw()` function
2. **Publish tag-relative yaw** — `tag_detector_adapter_node` already publishes the pose, just needs the new field
3. **Use in precision landing** — replace `req.yaw = 0.0` with a PID-controlled yaw command that drives `tag_relative_yaw` toward a target orientation (e.g., 0.0 = aligned with tag top edge)
4. **Integrate with ISS-013** — the HOLD_ABOVE_TAG → ALIGN_YAW phases (implemented in ISS-013) are the natural place to achieve both lateral offset AND yaw alignment before final descent

### Files involved

- `src/dbvf_msgs/msg/LandingTargetPose.msg` — add `float64 tag_yaw` field
- `src/dbvf_autonomy/dbvf_autonomy/tag_detector_adapter_node.py` — extract yaw from homography decomposition (lines 96-118)
- `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py` — consume `tag_yaw` in yaw hold logic (line 661)

---

## ISS-016: Drone pauses mid-descent during precision landing — DESCEND_OFFSET hovers at vz=0

**Date:** 2026-04-02, updated 2026-04-03
**Status:** Partially resolved — needs flight test

### Symptom

The drone visibly pauses (stops descending) above each landing zone before continuing down. This costs time during the competition sequence.

### Investigation and fixes applied

**Fix #1: Removed DESCEND_HOLD state (6-second hover)**

DESCEND_HOLD forced `vz=0.0` for `hold_stabilize_time` (configured at **6.0 seconds** in both sim and hardware configs). Removed the state entirely. The `hold_stabilize_time` parameter and the `_descend_hold()` handler were deleted from precision_landing_node.py and both config files.

**Fix #2: Altitude gate on DESCEND_COARSE → HOLD_ABOVE_TAG (added then removed)**

Attempted gating the transition to HOLD_ABOVE_TAG on `vs.alt_rel <= slow_descent_altitude` so the drone would keep descending during coarse approach and only pause at low altitude. The altitude gate was subsequently removed at user request — the transition now triggers on small tag confirmation time alone, regardless of altitude.

### Current descent flow (after all fixes)

| State | vz | Behavior |
|-------|----|----------|
| DESCEND_COARSE | `search_descent_rate` (0.3 m/s) | Actively descending, PID on primary tag |
| ↓ small tag confirmed (2s) | | |
| HOLD_ABOVE_TAG | 0.0 | Hold position, PID stabilization on secondary tag |
| ↓ hold_stabilize_time elapsed | | |
| ALIGN_YAW | 0.0 | Rotate to target yaw (auto-skips if tag_yaw unavailable) |
| ↓ yaw aligned or skipped | | |
| OFFSET_LATERAL | 0.0 | Move to lateral offset position |
| ↓ lateral error < offset_tolerance | | |
| DESCEND_FINAL | `final_descent_rate` (0.15 m/s) | Actively descending |

The 4-phase hold/align/offset/descend sequence (ISS-013) is intentionally paused at vz=0.0 for fine positioning. The key improvement is removing the original 6-second DESCEND_HOLD which had no positioning purpose — just a stabilization delay. The remaining hover time is determined by `hold_stabilize_time`, `yaw_alignment_hold_time`, and PID convergence for offset, which are all tunable.

### Files changed

- `precision_landing_node.py` — removed DESCEND_HOLD state, _descend_hold() method, hold_stabilize_time parameter declaration
- `config/sim_params.yaml` — removed original `hold_stabilize_time: 6.0` (ISS-013 re-added it at a lower default for HOLD_ABOVE_TAG)
- `config/hardware_params.yaml` — same
- `test_state_machine.py` — removed DESCEND_HOLD tests, updated transition tests
- `test_mission_state_machine.py` — updated WA reload trigger from DESCEND_HOLD to OFFSET_LATERAL
- `mission_state_machine.py` — WA reload triggers on OFFSET_LATERAL instead of DESCEND_HOLD

---

### Relationship to other issues

- **ISS-013** (4-phase final approach, IMPLEMENTED): The ALIGN_YAW phase is already implemented and auto-skips when `tag_yaw` is unavailable. Once ISS-015 adds the field, yaw alignment activates automatically via `getattr(target, 'tag_yaw', None)` in `_velocity_servo()`.
- **ISS-014** (yaw spin, FIXED): The `yaw=0.0` fix is a safe workaround; tag-relative yaw would be the proper solution that also enables orientation control
