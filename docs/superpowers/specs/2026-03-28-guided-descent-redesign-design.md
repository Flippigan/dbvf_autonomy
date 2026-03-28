# Guided Descent Redesign — Precision Landing with Visual Servoing

**Date:** 2026-03-28
**Status:** Draft
**Package:** `src/dbvf_autonomy/`, `src/dbvf_msgs/`
**Scope:** Replace ArduPilot LAND/PLND handoff with full GUIDED-mode descent using velocity-based visual servoing, dual-tag offset maneuvering, and small-tag search pattern

---

## 1. Overview

The current precision landing system uses a hybrid approach: GUIDED mode for approach and search, then handoff to ArduPilot's built-in PLND controller in LAND mode for the actual descent. This redesign removes the LAND mode handoff entirely and keeps the drone in GUIDED mode throughout, using velocity-based visual servoing (PID on AprilTag body-frame position) to control lateral correction and descent rate.

The key new capability is **small-tag offset maneuvering**: when the secondary AprilTag (ID 1, 0.15m) is detected at low altitude, the drone pauses descent, laterally offsets to a configurable position relative to the small tag (aligning over the payload reloading mechanism), then performs a slow final descent.

### Why This Change

- ArduPilot PLND has no concept of "offset from tag" — it always centers on the detected target
- The payload reloading mechanism may not be centered on the AprilTag; the offset needs to be tunable during physical testing
- Full GUIDED control allows pausing descent, performing search patterns, and controlling descent rate at each phase independently
- Removes dependency on ArduPilot PLND parameter tuning (PLND_LAG, PLND_EST_TYPE, etc.)

### Design Constraints

- All existing constraints from the original precision landing design still apply (vertical descent, Jetson transferable, swappable detection backend, shared MAVLink interface, extensible)
- The `tag_detector_adapter_node` boundary is preserved — it reports what it sees, the landing node decides what to do
- Existing tests must continue to pass; new states get new tests

---

## 2. State Machine

The FSM expands from 6 states to 10. The top-level flow remains recognizable:

```
IDLE
  │  Called via service with target GPS coords
  v
APPROACH
  │  GUIDED mode, fly to target lat/lon at current altitude
  │  Descend to approach_altitude
  │  Wait until within position_tolerance and at approach_altitude
  v
SEARCH
  │  Slow GUIDED descent at search_descent_rate
  │  Monitor for any tag detection
  │  If tag confirmed (tag_confirm_frames consecutive) → DESCEND_COARSE
  │  If altitude < min_search_altitude and no tag → ABORT_LAND
  │  If landing_timeout exceeded → ABORT_LAND
  v
DESCEND_COARSE
  │  Velocity-servo laterally on active tag (large or small), descend at search_descent_rate
  │  Track small tag detection duration
  │  If small tag detected continuously for small_tag_confirm_time → DESCEND_HOLD
  │  If altitude < descend_floor_altitude and no small tag → SMALL_TAG_SEARCH
  │  If all tags lost for tag_lost_timeout → SEARCH
  │  If landing_timeout exceeded → ABORT_LAND
  v
DESCEND_HOLD
  │  Hold altitude (vz = 0)
  │  Continue lateral servoing on small tag (centering, not offset yet)
  │  Hold for hold_stabilize_time
  │  → DESCEND_OFFSET
  v
DESCEND_OFFSET
  │  Servo to (offset_forward, offset_right) from small tag position
  │  PID target: (position_x - offset_forward, position_y - offset_right) → 0
  │  Hold altitude (vz = 0)
  │  When position error < offset_tolerance → DESCEND_FINAL
  │  If small tag lost for tag_lost_timeout → SEARCH
  │  If landing_timeout exceeded → ABORT_LAND
  v
DESCEND_FINAL
  │  Slow descent at final_descent_rate
  │  Continue lateral servoing (maintain offset if small tag visible, center if only large tag)
  │  Landing detection (disarmed OR alt < 0.1m with |vz| < 0.1) → LANDED
  v
LANDED
  │  Report success, return to IDLE

SMALL_TAG_SEARCH  (from DESCEND_COARSE when altitude floor reached)
  │  Hold altitude at descend_floor_altitude
  │  Sequential search pattern:
  │    1. Move forward search_radius at search_speed → return to center
  │    2. Move right search_radius → return to center
  │    3. Move backward search_radius → return to center
  │    4. Move left search_radius → return to center
  │  If small tag detected at any point → DESCEND_HOLD
  │  If full cycle completes without detection → DESCEND_FINAL (land in place)
  v

ABORT_LAND  (unchanged)
  │  Descend in place at final_descent_rate
  │  Report failure reason
  │  → LANDED when touchdown detected
```

### State Transition Summary

| From | To | Condition |
|------|----|-----------|
| IDLE | APPROACH | Service call with target GPS |
| APPROACH | SEARCH | Within position_tolerance and at approach_altitude |
| SEARCH | DESCEND_COARSE | Tag confirmed for tag_confirm_frames |
| SEARCH | ABORT_LAND | Below min_search_altitude or timeout |
| DESCEND_COARSE | DESCEND_HOLD | Small tag detected for small_tag_confirm_time |
| DESCEND_COARSE | SMALL_TAG_SEARCH | At descend_floor_altitude, no small tag |
| DESCEND_COARSE | SEARCH | All tags lost for tag_lost_timeout |
| DESCEND_COARSE | ABORT_LAND | Timeout |
| DESCEND_HOLD | DESCEND_OFFSET | hold_stabilize_time elapsed |
| DESCEND_OFFSET | DESCEND_FINAL | Position error < offset_tolerance |
| DESCEND_OFFSET | SEARCH | Small tag lost for tag_lost_timeout |
| DESCEND_OFFSET | ABORT_LAND | Timeout |
| DESCEND_FINAL | LANDED | Disarmed or alt < 0.1m |
| SMALL_TAG_SEARCH | DESCEND_HOLD | Small tag detected |
| SMALL_TAG_SEARCH | DESCEND_FINAL | Full cycle, no detection |
| ABORT_LAND | LANDED | Touchdown detected |

---

## 3. Visual Servo Controller

### PID Controller

A simple `PIDController` class used for lateral correction:

```python
class PIDController:
    def __init__(self, kp, ki, kd, output_limit):
        ...

    def update(self, error, dt):
        """Returns control output clamped to [-output_limit, output_limit]."""
        ...

    def reset(self):
        """Reset integral and previous error."""
        ...
```

Two instances: `pid_x` (forward axis) and `pid_y` (right axis).

### Control Loop (20 Hz)

Each tick in states `DESCEND_COARSE`, `DESCEND_HOLD`, `DESCEND_OFFSET`, `DESCEND_FINAL`, and `SMALL_TAG_SEARCH` (when returning to center or moving to excursion point):

1. Read latest tag position from `LandingTargetPose` (body-frame: position_x, position_y)
2. Compute error:
   - Centering (DESCEND_COARSE, DESCEND_HOLD): `error_x = position_x`, `error_y = position_y` — PID drives tag to image center
   - With offset (DESCEND_OFFSET, DESCEND_FINAL with small tag): `error_x = position_x - offset_forward`, `error_y = position_y - offset_right`
   - SMALL_TAG_SEARCH: PID not used for tag tracking — velocity commands are set directly by the search pattern logic (fixed direction at search_speed); PID resumes if small tag detected
3. PID outputs: `vx = pid_x.update(error_x, dt)`, `vy = pid_y.update(error_y, dt)`
4. State logic determines `vz`:
   - `DESCEND_COARSE`: `vz = search_descent_rate` (positive = down)
   - `DESCEND_HOLD`: `vz = 0`
   - `DESCEND_OFFSET`: `vz = 0`
   - `DESCEND_FINAL`: `vz = final_descent_rate`
5. Call `send_guided_velocity(vx, vy, vz)`

### When Tag Is Lost Momentarily

If the tag is not detected on a given tick but we haven't hit `tag_lost_timeout`:
- Send `vx = 0, vy = 0` (hold lateral position)
- `vz` per state logic (continue descent or hold, depending on state)
- Reset PID integrators to prevent windup during the gap

---

## 4. Velocity Command Interface

### New Service Definition

**`src/dbvf_msgs/srv/SendGuidedVelocity.srv`:**
```
float64 vx
float64 vy
float64 vz
---
bool success
string message
```

### mavlink_interface_node Implementation

New service server at `/dbvf/send_guided_velocity`. Handler sends:

```python
self.mav.set_position_target_local_ned_send(
    time_boot_ms,
    target_system, target_component,
    mavutil.mavlink.MAV_FRAME_BODY_NED,
    0b0000_11_0_111_000_111,   # type_mask: velocity only
    0, 0, 0,                    # position (ignored)
    vx, vy, vz,                # velocity (body frame)
    0, 0, 0,                    # acceleration (ignored)
    0, 0                        # yaw, yaw_rate (ignored)
)
```

- `MAV_FRAME_BODY_NED`: vx = forward, vy = right, vz = down
- Fire-and-forget (no ACK expected)
- ArduPilot requires velocity commands at least every 3s to keep moving; the 20 Hz loop provides this

### Updated Service Table

| Service | Type | Node |
|---------|------|------|
| `/dbvf/set_mode` | `dbvf_msgs/SetMode` | mavlink_interface |
| `/dbvf/arm_motors` | `dbvf_msgs/ArmMotors` | mavlink_interface |
| `/dbvf/send_guided_position` | `dbvf_msgs/SendGuidedPosition` | mavlink_interface |
| `/dbvf/send_guided_velocity` | `dbvf_msgs/SendGuidedVelocity` | mavlink_interface |
| `/dbvf/start_precision_landing` | `dbvf_msgs/StartPrecisionLanding` | precision_landing |

---

## 5. Tag Detector Adapter Changes

### Enable 3D Pose Passthrough

The `tag_detector_adapter_node` currently sets `position_valid = False`. Changes:

1. Read pose from `apriltag_ros` detection (`detection.pose.pose.pose.position`)
2. Apply camera-to-body frame transform:
   - `body_x (forward) = -camera_y`
   - `body_y (right) = camera_x`
   - `body_z (down) = camera_z`
3. Populate `position_x`, `position_y`, `position_z` on `LandingTargetPose`
4. Set `position_valid = True`

### Camera-to-Body Transform Configuration

The transform is configurable via YAML parameters for the case where camera mounting differs between platforms:

```yaml
tag_detector_adapter:
  ros__parameters:
    # Camera-to-body rotation (default: straight-down camera, X-forward aligned)
    cam_body_x_from: "-y"    # body X = negative camera Y
    cam_body_y_from: "x"     # body Y = camera X
    cam_body_z_from: "z"     # body Z = camera Z
```

The default matches the current hardmount configuration.

### No Other Changes

Dual-tag switching, debounce filter, angle computation — all unchanged. The adapter continues to report what it sees; the landing node decides what to do.

---

## 6. Configuration

### precision_landing parameters

```yaml
precision_landing:
  ros__parameters:
    # --- Existing (unchanged) ---
    approach_altitude: 8.0
    min_search_altitude: 1.0
    position_tolerance: 2.0
    tag_confirm_frames: 5
    tag_lost_timeout: 4.0
    landing_timeout: 60.0

    # --- Modified meaning ---
    search_descent_rate: 0.3         # Used in SEARCH and DESCEND_COARSE (m/s)

    # --- New: Descent phases ---
    descend_floor_altitude: 1.5      # Altitude to trigger small tag search if not found (meters)
    final_descent_rate: 0.15         # Slow descent in DESCEND_FINAL (m/s)

    # --- New: Small tag detection ---
    small_tag_confirm_time: 2.0      # Seconds of continuous small tag before DESCEND_HOLD
    hold_stabilize_time: 1.0         # Seconds to hold altitude before offset maneuver

    # --- New: Offset from small tag ---
    offset_forward: 0.0              # Body-frame forward offset from small tag (meters)
    offset_right: 0.0                # Body-frame right offset from small tag (meters)
    offset_tolerance: 0.05           # Position error to consider offset achieved (meters)

    # --- New: Small tag search pattern ---
    small_tag_search_radius: 0.5     # Lateral excursion distance (meters)
    small_tag_search_speed: 0.2      # Movement speed during search (m/s)

    # --- New: Visual servo PID ---
    servo_pid_p: 0.5
    servo_pid_i: 0.0
    servo_pid_d: 0.1
    servo_max_speed: 0.5             # Max lateral velocity output (m/s)
```

### ArduPilot Parameters

PLND parameters can remain enabled for telemetry/logging but are no longer used for flight control. The drone stays in GUIDED mode throughout descent, so PLND's lateral correction and retry logic are bypassed.

No new ArduPilot parameters required.

---

## 7. Data Flow (Updated)

```
[Camera Source]
  Sim: Gazebo bridge → /camera/image + /camera/camera_info
  Jetson: isaac_ros_argus_camera → NVMM GPU memory
    │
    v
[AprilTag Detector]
  Sim: apriltag_ros (CPU)
  Jetson: isaac_ros_apriltag (CUDA)
    │
    v
/apriltag/detections (AprilTagDetectionArray)
    │
    v
[tag_detector_adapter_node]
  Dual-tag switching + debounce
  Camera-to-body pose transform  ← NEW
    │
    ├──→ /dbvf/landing_target_pose (LandingTargetPose, now with position_valid=True)
    ├──→ /dbvf/tag_status (TagStatus)
    │
    v
[precision_landing_node]
  Extended FSM: IDLE → APPROACH → SEARCH → DESCEND_COARSE →
                DESCEND_HOLD → DESCEND_OFFSET → DESCEND_FINAL → LANDED
                                    (+ SMALL_TAG_SEARCH)
  PID visual servo controller  ← NEW
    │
    ├──→ /dbvf/landing_state (String — e.g. "DESCEND_OFFSET")
    │
    │    Calls services:
    │      /dbvf/set_mode (GUIDED only, no more LAND)
    │      /dbvf/send_guided_position (APPROACH, SEARCH)
    │      /dbvf/send_guided_velocity (all DESCEND_* and SMALL_TAG_SEARCH)  ← NEW
    v
[mavlink_interface_node]
  Owns pymavlink connection
    │
    ├──→ MAVLink SET_POSITION_TARGET_LOCAL_NED (velocity, 20 Hz)  ← NEW
    ├──→ MAVLink SET_POSITION_TARGET_GLOBAL_INT (position, APPROACH/SEARCH)
    ├──→ MAVLink SET_MODE (GUIDED only)
    ├──→ MAVLink HEARTBEAT (1 Hz)
    │
    ├←── MAVLink HEARTBEAT, GLOBAL_POSITION_INT ← ArduPilot
    │
    ├──→ /dbvf/vehicle_state (VehicleState, 10 Hz)
    └──→ /dbvf/heartbeat_status (Bool, 1 Hz)
```

**Key change:** No more `LANDING_TARGET` messages being sent to ArduPilot for PLND control. The `/dbvf/cmd/landing_target` topic and its subscription in `mavlink_interface_node` can be removed or retained for logging only.

---

## 8. Testing

### Existing Tests (unchanged, must continue to pass)

All 38 tests across 5 files. The pure-function signatures for `DebounceFilter`, `compute_angles`, `select_best_tag`, `build_landing_target_params`, and `ARDUPILOT_MODE_MAP` are not modified.

### Modified Tests

**`test_state_machine.py` — expand with new state transitions:**

| Test | Transition | Condition |
|------|-----------|-----------|
| test_descend_coarse_to_hold | DESCEND_COARSE → DESCEND_HOLD | Small tag detected for small_tag_confirm_time |
| test_descend_coarse_to_search_pattern | DESCEND_COARSE → SMALL_TAG_SEARCH | At descend_floor_altitude, no small tag |
| test_descend_coarse_tag_lost | DESCEND_COARSE → SEARCH | Tags lost for tag_lost_timeout |
| test_descend_hold_to_offset | DESCEND_HOLD → DESCEND_OFFSET | hold_stabilize_time elapsed |
| test_descend_offset_to_final | DESCEND_OFFSET → DESCEND_FINAL | Position error < offset_tolerance |
| test_descend_offset_tag_lost | DESCEND_OFFSET → SEARCH | Small tag lost for tag_lost_timeout |
| test_descend_final_to_landed | DESCEND_FINAL → LANDED | Landing detection |
| test_search_pattern_finds_tag | SMALL_TAG_SEARCH → DESCEND_HOLD | Small tag detected during excursion |
| test_search_pattern_gives_up | SMALL_TAG_SEARCH → DESCEND_FINAL | Full cycle, no detection |
| test_global_timeout_from_new_states | Any active → ABORT_LAND | landing_timeout exceeded |

### New Test Files

**`test_pid_controller.py`:**

| Test | Description |
|------|-------------|
| test_zero_error | Zero error → zero output |
| test_proportional | Step error → proportional response |
| test_integral_accumulation | Sustained error → growing output |
| test_integral_windup_clamp | Output clamped at servo_max_speed |
| test_derivative | Error change → derivative response |
| test_output_clamping | Large error → clamped output |
| test_reset | Reset clears integral and previous error |

**`test_pose_transform.py`:**

| Test | Description |
|------|-------------|
| test_tag_directly_below | Camera (0, 0, z) → body (0, 0, z) |
| test_tag_offset_camera_x | Camera (+x, 0, z) → body (0, +y, z) |
| test_tag_offset_camera_y | Camera (0, +y, z) → body (-x, 0, z) |
| test_transform_roundtrip | Known offsets produce expected body-frame values |

**`test_mavlink_messages.py` — extend:**

| Test | Description |
|------|-------------|
| test_velocity_type_mask | Correct bits for velocity-only command |
| test_velocity_frame | MAV_FRAME_BODY_NED used |

---

## 9. Files Changed

| File | Change |
|------|--------|
| `src/dbvf_msgs/srv/SendGuidedVelocity.srv` | **New** — velocity command service definition |
| `src/dbvf_msgs/CMakeLists.txt` | Add SendGuidedVelocity.srv to rosidl |
| `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py` | Extended FSM (4 new states + SMALL_TAG_SEARCH), PIDController class, velocity servoing logic |
| `src/dbvf_autonomy/dbvf_autonomy/tag_detector_adapter_node.py` | Enable pose passthrough, camera-to-body transform |
| `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py` | Add send_guided_velocity service, SET_POSITION_TARGET_LOCAL_NED |
| `src/dbvf_autonomy/config/sim_params.yaml` | New parameters (descent phases, PID, offset, search) |
| `src/dbvf_autonomy/config/jetson_params.yaml` | Same new parameters |
| `src/dbvf_autonomy/test/test_state_machine.py` | New transition tests for all new states |
| `src/dbvf_autonomy/test/test_pid_controller.py` | **New** — PID unit tests |
| `src/dbvf_autonomy/test/test_pose_transform.py` | **New** — camera-to-body transform tests |
| `src/dbvf_autonomy/test/test_mavlink_messages.py` | Extend with velocity command tests |

---

## 10. What's NOT Changing

- `tag_detector_adapter_node` dual-tag switching and debounce logic — unchanged
- `mavlink_interface_node` connection management, heartbeat, vehicle_state — unchanged
- `LandingTargetPose.msg`, `TagStatus.msg`, `VehicleState.msg` — unchanged
- `SetMode.srv`, `ArmMotors.srv`, `SendGuidedPosition.srv`, `StartPrecisionLanding.srv` — unchanged
- Launch files — unchanged (no new nodes, no removed nodes)
- apriltag_ros / isaac_ros_apriltag configuration — unchanged
- APPROACH and SEARCH state behavior — unchanged (still use send_guided_position)
