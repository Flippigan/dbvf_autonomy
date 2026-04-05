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
**Status:** IMPLEMENTED — tag_yaw extraction and publishing (2026-04-03). Active yaw alignment in precision_landing_node is ISS-013.

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

---

## ISS-017: Offsets have no effect — 4-phase landing never entered due to range_alt sentinel

**Date:** 2026-04-05
**Status:** FIXED
**Implementation plan:** `docs/superpowers/plans/2026-04-05-iss017-rangefinder-alt-fallback.md`

### Symptom

Setting `offset_forward` and `offset_right` to non-zero values (e.g., 10.0m each) in the `StartPrecisionLanding` service call has no effect on where the drone lands. The drone lands directly on the tag center regardless of offset values. The secondary tag IS visible in the debug camera view.

### Root Cause

`compute_preferred_tag_id()` (`precision_landing_node.py:84-95`) controls when the adapter switches from reporting the primary tag to the secondary tag. During DESCEND_COARSE, it only switches to secondary when:

```python
if (state == LandingState.DESCEND_COARSE
        and range_alt >= 0.0
        and range_alt <= slow_descent_altitude):   # 2.0m
    return secondary_tag_id
return primary_tag_id
```

`range_alt` initializes to **-1.0** (sentinel, `mavlink_interface_node.py:27,137`) and only updates when a `RANGEFINDER` MAVLink message arrives from ArduPilot. If RANGEFINDER messages are not received (e.g., SITL stream rates not configured, or MAVProxy not forwarding), `range_alt` stays -1.0 for the entire flight.

**Chain of failure:**

| Step | What happens |
|------|-------------|
| 1 | `range_alt` = -1.0 → `compute_preferred_tag_id` returns **primary** (1) throughout DESCEND_COARSE |
| 2 | Adapter receives preferred=1 → `select_best_tag` picks primary → `active_tag_id` = 1 |
| 3 | FSM's `small_tag_detected` = (`active_tag_id == secondary_tag_id`) = (`1 == 2`) = **False** |
| 4 | `small_tag_first_seen` timer never starts → never transitions to HOLD_ABOVE_TAG |
| 5 | Drone descends through DESCEND_COARSE until `_is_landed()` triggers → LANDED |
| 6 | OFFSET_LATERAL and DESCEND_FINAL (where offsets are applied) are never reached |

The secondary tag IS being detected by apriltag_ros (visible in debug view), but the adapter never reports it as `active_tag_id` because the preferred tag never switches from primary.

### How to verify

While the drone is descending, check the rangefinder reading:
```bash
ros2 topic echo /dbvf/vehicle_state --field range_alt
```
If it shows **-1.0** throughout, RANGEFINDER MAVLink messages are not arriving.

### Where offsets are applied (for reference)

| State | `use_offset` in info dict |
|-------|--------------------------|
| DESCEND_COARSE | not set (defaults `False`) |
| HOLD_ABOVE_TAG | `False` |
| ALIGN_YAW | `False` |
| **OFFSET_LATERAL** | **`True`** |
| **DESCEND_FINAL** | **`True`** (if small tag detected) |

Offsets are consumed in `_velocity_servo()` at `precision_landing_node.py:688-690`:
```python
if use_offset:
    error_x -= self.offset_forward
    error_y -= self.offset_right
```

### Possible fixes (not yet implemented)

1. **Add `alt_rel` fallback** — if `range_alt` is sentinel (-1.0), use `vs.alt_rel` (barometric) in `compute_preferred_tag_id` so the preferred tag switch doesn't depend entirely on rangefinder availability
2. **Request RANGEFINDER stream** — send `REQUEST_DATA_STREAM` or set `SRn_EXTRA3` parameter in ArduPilot to ensure RANGEFINDER messages are forwarded through MAVProxy
3. **Both** — fallback for robustness + stream config for accuracy

### Files involved

- `precision_landing_node.py:84-95` — `compute_preferred_tag_id()` range_alt gate
- `mavlink_interface_node.py:27,137,262-263` — RANGE_ALT_SENTINEL, RANGEFINDER handling
- `config/gazebo-iris-hardmount.parm:28-31` — RNGFND1 configured but stream rate not set

### Resolution

Two-pronged fix:

1. **`alt_rel` fallback in `compute_preferred_tag_id()`** — when `range_alt` is sentinel (-1.0), the function now falls back to `alt_rel` (barometric altitude from `GLOBAL_POSITION_INT`). This ensures the preferred tag switches to secondary during DESCEND_COARSE even without rangefinder data, enabling the 4-phase landing sequence.

2. **`SR0_EXTRA3 10` in `gazebo-iris-hardmount.parm`** — explicitly requests RANGEFINDER messages at 10Hz in SITL. When rangefinder data is available, it is preferred over barometric altitude (more accurate near ground).

### Files changed

| File | Change |
|------|--------|
| `precision_landing_node.py:84-102` | Added `alt_rel` parameter to `compute_preferred_tag_id()`, fallback logic |
| `precision_landing_node.py:655-659` | Pass `vs.alt_rel` at call site |
| `test_state_machine.py` | 4 new tests for alt_rel fallback |
| `gazebo-iris-hardmount.parm` | Added `SR0_EXTRA3 10` |

### Relationship to other issues

- **ISS-013** (4-phase landing): The 4-phase sequence works correctly IF entered. This issue prevents entry.
- **ISS-016** (mid-descent pause): The pause in HOLD_ABOVE_TAG is intentional for fine positioning — but it's never reached due to this bug.
- **ISS-018**: Despite this fix, the symptom persists — see ISS-018 below.

---

## ISS-018: 4-phase landing still not entered after ISS-017 fix

**Date:** 2026-04-05
**Status:** FIXED (2026-04-05)
**Related:** ISS-017 (fix applied but symptom persists)

### Symptom

After applying the ISS-017 fix (`alt_rel` fallback in `compute_preferred_tag_id()` + `SR0_EXTRA3 10` stream rate param), the drone still:

1. Does **not pause** above the smaller tag (HOLD_ABOVE_TAG never entered)
2. Does **not move** to the offset specified in `StartPrecisionLanding` (offset_forward=0.5, offset_right=0.5)
3. FSM goes APPROACH → SEARCH → (stays in SEARCH for extended period) → eventually DESCEND_COARSE → LANDED

### Diagnostic data collected (2026-04-05)

Monitored `/dbvf/vehicle_state`, `/dbvf/tag_status`, `/dbvf/cmd/preferred_tag_id`, `/dbvf/landing_state` during a full landing attempt:

| Observation | Data |
|-------------|------|
| `range_alt` | **Working** (not sentinel). Decreases from ~6.7m to ~1.2m, then holds. SR0_EXTRA3 fix effective. |
| `alt_rel` | **Working**. Tracks range_alt, decreases from ~6.7m to ~1.0m (min_search_altitude). |
| `preferred_tag_id` | **Always 1** for entire capture. Never switches to 2 because FSM never enters DESCEND_COARSE during capture window. |
| `tag_status` | **No tags detected** at >3m altitude. **Flickering** detection of tag 1 at 2-3m (true/false alternating). **Stable** tag 1 detection at ~1.0m. **Tag 2 never detected.** |
| `landing_state` | APPROACH → SEARCH. Stays in SEARCH. |
| `active_tag_id` | Always 1 or -1. **Never 2.** |

### Root cause

**Two compounding issues:**

#### 1. Stale apriltag_ros config in `precision_landing_sim.launch.py`

```python
# CURRENT (wrong):
'tag.ids': [0, 1],
'tag.sizes': [0.6, 0.15],

# CORRECT (matches world and other launch files):
'tag.ids': [1, 2],
'tag.sizes': [0.15, 0.10],
```

The Gazebo world (`iris_runway.sdf`) has tags **ID 1** (0.15m) and **ID 2** (0.05m model / 0.10m in config). Tag ID 0 does **not exist** — it was removed when the tag layout was restructured. Three other launch files (`mission_sim`, `precision_landing_real`, `mission_real`) already have the correct `[1, 2]` config. Only `precision_landing_sim.launch.py` was missed.

While apriltag_ros still detects tag 2 even without it in `tag.ids` (it only affects pose size estimation, not detection filtering), the wrong default size (0.6m instead of 0.05m) causes wildly incorrect pose estimates that may confuse downstream angle calculations.

#### 2. Secondary tag model is 0.05m — too small for reliable detection

The `Apriltag36_11_00002` Gazebo model is 0.05m (5cm). At the altitudes where the preferred tag switches (≤2.0m), this tag is ~16 pixels on camera — marginal for AprilTag detection. The flickering detection pattern observed for tag 1 (0.15m) at 2-3m confirms that even larger tags struggle at these distances. Tag 2 at 0.05m is never detected during the landing.

Note: `sim_params.yaml` and other launch files reference `secondary_tag_size: 0.05` / `tag.sizes: [0.15, 0.10]` — there's a discrepancy between the model (0.05m) and config (0.10m) that needs resolving.

### Also found: `jetson_params.yaml` has stale tag IDs

```yaml
# CURRENT (wrong):
primary_tag_id: 0
secondary_tag_id: 1

# CORRECT:
primary_tag_id: 1
secondary_tag_id: 2
```

### Fix needed

1. **Fix `precision_landing_sim.launch.py`**: Change `tag.ids: [0, 1]` → `[1, 2]`, `tag.sizes: [0.6, 0.15]` → `[0.15, 0.10]`
2. **Fix `jetson_params.yaml`**: Change tag IDs from 0/1 to 1/2
3. **Increase secondary tag model size**: Enlarge `Apriltag36_11_00002` from 0.05m to 0.10m (matching config and improving detectability)
4. **Reconcile tag size references**: Ensure model, configs, and launch files all agree on 0.10m for secondary tag

### Files involved

- `launch/precision_landing_sim.launch.py:27-32` — stale `tag.ids: [0, 1]`
- `config/jetson_params.yaml` — stale `primary_tag_id: 0, secondary_tag_id: 1`
- `src/ardupilot_gazebo/models/Apriltag36_11_00002/` — 0.05m model, needs 0.10m
- `config/sim_params.yaml:14` — `secondary_tag_size: 0.05` (should be 0.10)

### Relationship to other issues

- **ISS-017** (range_alt fallback): ISS-017 fix is correct and verified working (range_alt is now valid). But the symptom persists because tag 2 is never detected — a separate issue.
- **ISS-013** (4-phase landing): The 4-phase sequence would work IF tag 2 were detected. This issue blocks entry.

### Resolution (2026-04-05)

All config and model files updated to agree on: primary tag = ID 1 @ 0.15m, secondary tag = ID 2 @ 0.10m.

| File | Before | After |
|------|--------|-------|
| `launch/precision_landing_sim.launch.py` | `tag.ids: [0, 1]`, `tag.sizes: [0.6, 0.15]` | `tag.ids: [1, 2]`, `tag.sizes: [0.15, 0.10]` |
| `config/sim_params.yaml` | `secondary_tag_size: 0.05` | `secondary_tag_size: 0.10` |
| `config/hardware_params.yaml` | `secondary_tag_size: 0.05` | `secondary_tag_size: 0.10` |
| `config/jetson_params.yaml` | `primary_tag_id: 0`, `secondary_tag_id: 1`, sizes 0.6/0.15 | IDs 1/2, sizes 0.15/0.10 |
| `models/Apriltag36_11_00002/model.sdf` | `0.05 0.05 0.001` | `0.10 0.10 0.001` |

Additional find: `hardware_params.yaml` had the same `secondary_tag_size: 0.05` stale value — included in fix.

---

## ISS-019: 4-phase landing still not activating after ISS-018 config fix

**Date:** 2026-04-05
**Status:** Open
**Related:** ISS-018 (config fix applied), ISS-017 (alt_rel fallback applied), ISS-013 (4-phase landing design)

### Symptom

After applying the ISS-018 config fix (all tag IDs, sizes, and Gazebo model now consistent), the drone still:

1. Does **not stop** above the secondary (smaller) tag — HOLD_ABOVE_TAG state never entered
2. Does **not apply** the lateral offset — offset_forward=1.0, offset_right=1.0 have no visible effect on landing position

### Reproduction

```bash
# Gazebo + SITL running, precision landing stack launched
ros2 service call /dbvf/start_precision_landing dbvf_msgs/srv/StartPrecisionLanding \
  "{target_lat: -35.3633033, target_lon: 149.1657423, offset_forward: 1.0, offset_right: 1.0}"
```

### What has already been ruled out

- **Stale launch config (ISS-018):** All four launch files now have `tag.ids: [1, 2]`, `tag.sizes: [0.15, 0.10]`. Verified consistent.
- **Stale YAML config (ISS-018):** All three config files have `secondary_tag_size: 0.10`, correct tag IDs 1/2.
- **Undersized Gazebo model (ISS-018):** Secondary tag model enlarged from 0.05m to 0.10m.
- **Missing alt_rel stream (ISS-017):** `SR0_EXTRA3 10` added, `alt_rel` fallback in `compute_preferred_tag_id()`.

### Investigation needed

The config layer is now correct. The issue is likely in the runtime behavior — possible areas:

1. **Is tag ID 2 actually being detected by apriltag_ros?** Check `/apriltag/detections` topic during descent to confirm tag 2 appears.
2. **Is the tag_detector_adapter switching to tag 2?** Check `/dbvf/tag_status` for `active_tag_id: 2` during descent.
3. **Is DESCEND_COARSE transitioning to HOLD_ABOVE_TAG?** The transition requires the secondary tag to be the active tag AND `range_alt ≤ slow_descent_altitude`. Check `/dbvf/landing_state` for state transitions.
4. **Is the offset being stored on the node?** The `StartPrecisionLanding` service stores `offset_forward`/`offset_right` on `self` — verify the service response confirms acceptance.
5. **Are the OFFSET_LATERAL PID commands actually being sent?** If HOLD_ABOVE_TAG is never reached, OFFSET_LATERAL is never reached either, and offsets are never applied.

### Files likely involved

- `dbvf_autonomy/precision_landing_node.py` — FSM transitions, DESCEND_COARSE → HOLD_ABOVE_TAG guard, offset storage
- `dbvf_autonomy/tag_detector_adapter_node.py` — tag switching, debounce filter, preferred tag subscription
- `config/sim_params.yaml` — `slow_descent_altitude`, debounce thresholds
