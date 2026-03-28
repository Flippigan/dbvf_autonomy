# CLAUDE.md — dbvf_autonomy

## Package Purpose

ROS2 autonomy package for the VFS DBVF (Design-Build-Vertical-Flight) competition. Currently implements AprilTag-based precision landing. Will expand to include payload pickup, drop zone navigation, and payload delivery.

## Package Type

Mixed `ament_cmake` + `ament_cmake_python` package. CMake handles build system and rosidl (via dependency on `dbvf_msgs`), Python handles node implementations.

## Critical: Message Imports

Messages and services live in the **separate `dbvf_msgs` package** (not here) due to rosidl namespace collision. Always use:
```python
from dbvf_msgs.msg import LandingTargetPose, TagStatus, VehicleState
from dbvf_msgs.srv import SetMode, ArmMotors, SendGuidedPosition, StartPrecisionLanding
```
Never `from dbvf_autonomy.msg import ...` — that path does not exist.

## Nodes

### mavlink_interface_node
- Single owner of pymavlink TCP connection to ArduPilot SITL
- Publishes: `/dbvf/vehicle_state` (10Hz), `/dbvf/heartbeat_status` (1Hz)
- Subscribes: `/dbvf/cmd/landing_target` (forwards as MAVLink LANDING_TARGET)
- Services: `/dbvf/set_mode`, `/dbvf/arm_motors`, `/dbvf/send_guided_position`
- Config: connection_string, source_system, source_component, heartbeat_rate, vehicle_state_rate

### tag_detector_adapter_node
- Bridges apriltag_ros detections to custom LandingTargetPose messages
- Dual-tag switching: primary (ID 0, 0.6m) preferred, secondary (ID 1, 0.15m) fallback
- DebounceFilter: rolling buffer (30 frames), 80% threshold to switch
- Computes angle_x/angle_y from camera intrinsics (CameraInfo K matrix)
- Publishes: `/dbvf/landing_target_pose`, `/dbvf/tag_status`

### precision_landing_node
- State machine: IDLE → APPROACH → SEARCH → DESCEND → LANDED (+ ABORT_LAND)
- APPROACH: GUIDED mode, fly to target GPS coords
- SEARCH: Slow guided descent (0.3 m/s), wait for tag confirmation (5 consecutive frames)
- DESCEND: LAND mode, forward LANDING_TARGET to ArduPilot PLND at 20Hz
- Tag lost timeout (4s) → back to SEARCH; global timeout (60s) → ABORT_LAND
- Triggered via `/dbvf/start_precision_landing` service

## Testing

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon test --packages-select dbvf_autonomy
colcon test-result --verbose
```

38 tests across 5 files:
- `test_mavlink_messages.py` — mode map, landing target params
- `test_debounce_filter.py` — rolling buffer debounce logic
- `test_angle_computation.py` — pixel-to-angle atan2 conversion
- `test_tag_selection.py` — primary/secondary tag priority
- `test_state_machine.py` — all FSM state transitions

Tests require ROS2 environment sourced (`rclpy` import at module level).

## Build

```bash
colcon build --packages-select dbvf_msgs dbvf_autonomy
```

## Design Documentation

- Spec: `docs/superpowers/specs/2026-03-27-precision-landing-design.md`
- Plan: `docs/superpowers/plans/2026-03-27-precision-landing.md`
- Log: `docs/superpowers/Implimentation Log/2026-03-27-precision-landing-implementation.md`
