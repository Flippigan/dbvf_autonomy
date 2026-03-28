# Preferred Tag Coordination — ISS-012 Fix

**Date:** 2026-03-28
**Issue:** ISS-012 — Drone does not switch landing target from large tag to small tag after detection
**Status:** Design approved, pending implementation

## Problem

The drone stays centered on the primary AprilTag (ID 0, 0.6m) even after the FSM confirms the secondary tag (ID 1, 0.15m) and transitions to DESCEND_HOLD/DESCEND_OFFSET.

### Root Cause

`select_best_tag()` in `tag_detector_adapter_node.py:56-62` unconditionally prefers the primary tag when both tags are visible. Both tags are within the camera's 114.6 HFOV at all descent altitudes (they're 0.5m apart; ground coverage at 1m is ~3.1m). This causes the debounce to revert to primary within ~0.8s whenever the drone stabilizes (e.g., entering DESCEND_HOLD with vz=0), because primary detection recovers and `select_best_tag` feeds primary as the candidate on every frame.

The adapter has no knowledge of the FSM state, so it independently reverts to primary even though the FSM has confirmed secondary and is trying to center on it.

## Solution

Add a preferred tag coordination topic so the precision_landing_node can tell the adapter which tag to prioritize. The adapter's `select_best_tag` respects the preference when the preferred tag is detected, falling back to current primary-first logic otherwise. A safety check in `_velocity_servo` rejects stale target data from the wrong tag during the debounce transition window.

## Design

### 1. Preferred Tag Topic

- **Topic:** `/dbvf/cmd/preferred_tag_id`
- **Type:** `std_msgs/Int32`
- **Publisher:** `precision_landing_node` at 20Hz (every control loop tick)
- **Subscriber:** `tag_detector_adapter_node`

Values published by FSM state:

| FSM States | Value | Reason |
|------------|-------|--------|
| IDLE, APPROACH, SEARCH, DESCEND_COARSE | `0` (primary) | Large tag for coarse guidance |
| DESCEND_HOLD, DESCEND_OFFSET, DESCEND_FINAL | `1` (secondary) | Fine positioning on small tag |
| SMALL_TAG_SEARCH | `1` (secondary) | Actively looking for small tag |
| ABORT_LAND | `0` (primary) | Best available reference |

Only published when FSM is active (not IDLE), matching the existing control loop guard.

### 2. Adapter Changes (`tag_detector_adapter_node.py`)

**`select_best_tag` gains a `preferred_id` parameter:**

```python
def select_best_tag(detections_by_id, primary_id, secondary_id, preferred_id=None):
    if preferred_id is not None and preferred_id in detections_by_id:
        return preferred_id, detections_by_id[preferred_id]
    if primary_id in detections_by_id:
        return primary_id, detections_by_id[primary_id]
    if secondary_id in detections_by_id:
        return secondary_id, detections_by_id[secondary_id]
    return None, None
```

Priority order: preferred (if detected) > primary (if detected) > secondary (if detected) > None.

Default `preferred_id=None` preserves existing behavior for all current callers.

**Node changes:**
- New subscriber on `/dbvf/cmd/preferred_tag_id`, stores `self.preferred_tag_id = None`
- `_detection_cb` passes `self.preferred_tag_id` to `select_best_tag`
- No changes to debounce, `_publish_target`, or TagStatus logic

The debounce naturally transitions because the candidate is now consistently secondary when both tags are visible and preferred=1. Within ~0.8s (80% of 30-frame buffer) the debounce locks to secondary.

### 3. Safety Check in `_velocity_servo` (`precision_landing_node.py`)

After reading `self.latest_target`, add a tag ID guard for states that require the secondary tag:

```python
target = self.latest_target
expected_secondary = (state in (LandingState.DESCEND_HOLD,
                                LandingState.DESCEND_OFFSET,
                                LandingState.DESCEND_FINAL))
if expected_secondary and target is not None and target.tag_id != self.fsm.config.get('secondary_tag_id', 1):
    target = None
```

When `target` is None, the existing else branch fires: `vx=vy=0`, PID resets. The drone holds position until the adapter's debounce catches up (~0.8s). This prevents a brief servo-toward-primary glitch during the debounce transition window.

### 4. Publisher in `precision_landing_node.py`

New publisher created in `__init__`:

```python
self.preferred_tag_pub = self.create_publisher(Int32, '/dbvf/cmd/preferred_tag_id', 10)
```

Published every tick at the end of `_control_loop`, before the state string publication:

```python
preferred = Int32()
preferred.data = 1 if state in (LandingState.DESCEND_HOLD, LandingState.DESCEND_OFFSET,
                                LandingState.DESCEND_FINAL, LandingState.SMALL_TAG_SEARCH) else 0
self.preferred_tag_pub.publish(preferred)
```

## Files Changed

| File | Changes |
|------|---------|
| `src/dbvf_autonomy/dbvf_autonomy/tag_detector_adapter_node.py` | `select_best_tag` gains `preferred_id` param; new subscriber; pass preferred to `select_best_tag` in `_detection_cb` |
| `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py` | New publisher; publish preferred tag every tick in `_control_loop`; tag ID safety check in `_velocity_servo` |
| `src/dbvf_autonomy/test/test_tag_selection.py` | 4 new tests for preferred tag selection |
| `src/dbvf_autonomy/test/test_state_machine.py` or new file | 2 new tests for velocity servo tag ID guard |

No new messages, services, or config changes. Uses `std_msgs/Int32`. Tag IDs are already parameterized as `primary_tag_id` and `secondary_tag_id`.

## Tests

### New `select_best_tag` tests (in `test_tag_selection.py`)

- `test_preferred_secondary_when_both_visible` — preferred=1, both detected -> returns secondary
- `test_preferred_primary_when_both_visible` — preferred=0, both detected -> returns primary
- `test_preferred_not_detected_falls_back` — preferred=1, only primary detected -> returns primary
- `test_preferred_none_uses_default` — preferred=None, both detected -> returns primary (existing behavior)

### New velocity servo safety check tests

- `test_velocity_servo_rejects_wrong_tag_in_hold` — DESCEND_HOLD with target.tag_id=0 -> target treated as None (hold position)
- `test_velocity_servo_accepts_correct_tag_in_hold` — DESCEND_HOLD with target.tag_id=1 -> normal PID output

### Existing tests

All 5 existing `test_tag_selection.py` tests pass unchanged (default `preferred_id=None` preserves behavior).

## Data Flow After Fix

```
precision_landing_node (FSM state)
    -> /dbvf/cmd/preferred_tag_id (20Hz, Int32)
    -> tag_detector_adapter_node (select_best_tag respects preference)
    -> /dbvf/landing_target_pose (secondary tag position when preferred)
    -> precision_landing_node (_velocity_servo, with tag_id safety check)
    -> PID centers drone on secondary tag
```
