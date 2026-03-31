# CLAUDE.md — dbvf_autonomy

## Package Purpose

ROS2 autonomy package for the VFS DBVF (Design-Build-Vertical-Flight) competition. Implements AprilTag-based precision landing and a full mission sequencer for the competition sequence (FM-1 through FM-3 + RTH).

## Package Type

Mixed `ament_cmake` + `ament_cmake_python` package. CMake handles build system and rosidl (via dependency on `dbvf_msgs`), Python handles node implementations.

## Critical: Message Imports

Messages and services live in the **separate `dbvf_msgs` package** (not here) due to rosidl namespace collision. Always use:
```python
from dbvf_msgs.msg import LandingTargetPose, TagStatus, VehicleState
from dbvf_msgs.srv import SetMode, ArmMotors, SendGuidedPosition, StartPrecisionLanding, Takeoff, DoSetServo
from dbvf_msgs.srv import StartMission, ResumeMission, AbortMission  # mission sequencer
```
Never `from dbvf_autonomy.msg import ...` — that path does not exist.

## Nodes

### mavlink_interface_node
- Single owner of pymavlink TCP connection to ArduPilot SITL
- Publishes: `/dbvf/vehicle_state` (10Hz), `/dbvf/heartbeat_status` (1Hz)
- Subscribes: `/dbvf/cmd/landing_target` (forwards as MAVLink LANDING_TARGET)
- Services: `/dbvf/set_mode`, `/dbvf/arm_motors`, `/dbvf/send_guided_position`, `/dbvf/send_guided_velocity`, `/dbvf/takeoff`, `/dbvf/do_set_servo`
- Config: connection_string, source_system, source_component, heartbeat_rate, vehicle_state_rate

### tag_detector_adapter_node
- Bridges apriltag_ros detections to custom LandingTargetPose messages
- Dual-tag switching: primary (ID 0, 0.6m) and secondary (ID 1, 0.15m)
- `select_best_tag()`: preferred (if detected) > primary > secondary > None
- DebounceFilter: rolling buffer (30 frames), 80% threshold to switch; immediate bypass when candidate matches preferred tag
- Subscribes: `/dbvf/cmd/preferred_tag_id` (Int32) for FSM-driven tag coordination
- Computes angle_x/angle_y from camera intrinsics (CameraInfo K matrix)
- Publishes: `/dbvf/landing_target_pose`, `/dbvf/tag_status`

### precision_landing_node
- State machine: IDLE → APPROACH → SEARCH → DESCEND_COARSE → DESCEND_HOLD → DESCEND_OFFSET → DESCEND_FINAL → LANDED (+ SMALL_TAG_SEARCH, ABORT_LAND)
- APPROACH: GUIDED mode, fly to target GPS coords
- SEARCH: Slow guided descent (0.3 m/s), wait for tag confirmation (5 consecutive frames)
- DESCEND_COARSE: PID visual servo on primary tag; publishes preferred_id=1 when range_alt ≤ slow_descent_altitude (2.0m)
- DESCEND_HOLD/OFFSET/FINAL: PID visual servo on secondary tag with safety guard rejecting primary tag data
- `compute_preferred_tag_id()`: pure function for preferred tag logic
- Publishes: `/dbvf/cmd/preferred_tag_id` (Int32, 20Hz) for adapter coordination
- Tag lost timeout (4s) → back to SEARCH; global timeout (60s) → ABORT_LAND
- Triggered via `/dbvf/start_precision_landing` service

### mission_sequencer_node
- 17-state linear FSM: IDLE → PREFLIGHT_CHECK → TAKEOFF_H → TRANSIT_H_TO_L → LAND_L → WAIT_FLAGGER → TAKEOFF_L → TRANSIT_TO_DROP → DROP_PAYLOAD → TRANSIT_TO_WA → LAND_WA → TAKEOFF_WA → TRANSIT_TO_DROP_2 → DROP_PAYLOAD_2 → TRANSIT_TO_H → LAND_H → COMPLETE (+ ABORT)
- Orchestrates full DBVF competition: FM-1, FM-2, FM-3, RTH
- Pure Python state machine (`mission_state_machine.py`) wrapped by ROS2 node
- Delegates to: `set_mode`, `arm_motors`, `takeoff`, `send_guided_position`, `start_precision_landing`, `do_set_servo`
- Publishes: `/dbvf/mission_state` (10Hz), `/dbvf/mission_phase` (on change)
- Services: `/dbvf/start_mission`, `/dbvf/resume_mission`, `/dbvf/abort_mission`
- Config: `config/mission_params.yaml` — GPS waypoints, flight params, servo config, safety timeouts
- **Important:** ROS2 message objects use `__slots__` — cannot set dynamic attributes. The node wraps `VehicleState` into a plain Python object before passing to the FSM.

### mission_helpers.py (not a node)
- Pure functions: `ft_to_m`, `m_to_ft`, `haversine_distance_m`, `is_within_tolerance`
- Config validation: `validate_mission_config`, `DEFAULT_MISSION_CONFIG`
- No ROS2 dependencies — fully unit-testable

## Testing

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon test --packages-select dbvf_autonomy
colcon test-result --verbose
```

151 tests across 12 files:
- `test_mavlink_messages.py` — mode map, landing target params
- `test_debounce_filter.py` — rolling buffer debounce logic
- `test_angle_computation.py` — pixel-to-angle atan2 conversion
- `test_tag_selection.py` — primary/secondary tag priority
- `test_state_machine.py` — precision landing FSM transitions
- `test_pid_controller.py` — PID controller logic
- `test_pose_transform.py` — pose transformation
- `test_tag_pose_estimation.py` — tag pose estimation
- `test_descent_rate_clamp.py` — descent rate clamping
- `test_mission_helpers.py` — ft/m conversion, haversine, tolerance
- `test_mission_config.py` — config validation, defaults
- `test_mission_state_machine.py` — all 17 mission states + ABORT transitions

Tests are pure-Python (no ROS2 runtime needed) but require ROS2 environment sourced for imports.

## Build

```bash
colcon build --packages-select dbvf_msgs dbvf_autonomy
```

## Design Documentation

- Precision landing spec: `docs/superpowers/specs/2026-03-27-precision-landing-design.md`
- Mission sequencer spec: `docs/superpowers/specs/2026-03-29-mission-sequencer-design.md`
- Mission sequencer plan: `docs/superpowers/plans/2026-03-29-mission-sequencer.md`
- RC mission control plan: `docs/superpowers/plans/2026-03-30-rc-mission-control.md`
- Bug log: `docs/superpowers/Log/`

## Team Integration Docs

- **GUI integration guide:** `docs/Features/GUI/mission_sequencer_gui_integration.md` — GPS waypoints the GUI must set, all `/dbvf/mission_state` and `/dbvf/mission_phase` values for feedback display, available services (start/resume/abort), and suggested GUI layout
