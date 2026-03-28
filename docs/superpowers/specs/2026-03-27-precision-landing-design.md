# Precision Landing System Design

**Date:** 2026-03-27
**Status:** Approved
**Package:** `src/dbvf_autonomy/`
**Scope:** AprilTag-based precision landing for VFS DBVF competition, with sim/Jetson transferability

---

## 1. Overview

A ROS2 precision landing system that uses AprilTag detection to guide an ArduPilot drone to a marked landing pad. The system runs identically in Gazebo simulation and on NVIDIA Jetson Orin hardware — only the launch file and config YAML differ between platforms.

The system uses a hybrid landing approach: custom GUIDED-mode positioning for approach and tag search, then handoff to ArduPilot's built-in precision landing controller (PLND) in LAND mode once the tag is confirmed.

### Design Constraints

- **Vertical descent only** — competition rules require straight-down descent (no lateral search patterns)
- **Jetson transferable** — all node code must run unchanged on both dev machine (sim) and Jetson Orin
- **Swappable detection backend** — `apriltag_ros` in sim, `isaac_ros_apriltag` on Jetson
- **Shared MAVLink interface** — single pymavlink connection shared by all future competition nodes
- **Extensible** — package will later include payload pickup, drop zone navigation, and payload drop

---

## 2. Package Structure

```
src/dbvf_autonomy/
├── dbvf_autonomy/
│   ├── __init__.py
│   ├── mavlink_interface_node.py
│   ├── tag_detector_adapter_node.py
│   └── precision_landing_node.py
├── config/
│   ├── sim_params.yaml
│   └── jetson_params.yaml
├── launch/
│   ├── precision_landing_sim.launch.py
│   └── precision_landing_jetson.launch.py
├── srv/
│   ├── SetMode.srv
│   ├── ArmMotors.srv
│   ├── SendGuidedPosition.srv
│   └── StartPrecisionLanding.srv
├── msg/
│   ├── LandingTargetPose.msg
│   ├── TagStatus.msg
│   └── VehicleState.msg
├── package.xml
├── setup.py
├── setup.cfg
└── resource/dbvf_autonomy
```

### Dependencies

- `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs` — standard ROS2
- `apriltag_msgs` — from `apriltag_ros` package (AprilTagDetectionArray)
- `pymavlink` — MAVLink communication (pip install)

The package never imports or depends on the detection backend directly. It only consumes `AprilTagDetectionArray` messages on a configurable topic name. The launch file wires the correct detector.

---

## 3. Node Architecture

### 3.1 MAVLink Interface Node (`mavlink_interface_node.py`)

Single owner of the pymavlink connection. All other nodes interact with ArduPilot through this node's services and topics.

**Connection:**
- Sim: `tcp:127.0.0.1:5760` (from `sim_params.yaml`)
- Jetson: `/dev/ttyTHS1` at 921600 baud (from `jetson_params.yaml`)
- Automatic reconnection with backoff on connection loss

**Published Topics:**

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/dbvf/vehicle_state` | `VehicleState` | 10 Hz | Mode, armed, lat/lon/alt, relative alt, velocity, heading |
| `/dbvf/heartbeat_status` | `std_msgs/Bool` | 1 Hz | Whether ArduPilot heartbeat is being received |

**Subscribed Topics:**

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/dbvf/cmd/landing_target` | `LandingTargetPose` | Up to 30 Hz | Forwarded as MAVLink LANDING_TARGET |

**Services:**

| Service | Type | Description |
|---------|------|-------------|
| `/dbvf/set_mode` | `SetMode` | Switch flight mode (GUIDED, LAND, AUTO, LOITER, RTL) |
| `/dbvf/arm_motors` | `ArmMotors` | Arm or disarm. Waits for ACK, returns success/failure |
| `/dbvf/send_guided_position` | `SendGuidedPosition` | Send GPS target in GUIDED mode via SET_POSITION_TARGET_GLOBAL_INT |

**Internal behavior:**
- Sends MAVLink heartbeat at 1 Hz (required to prevent GCS failsafe)
- Parses incoming MAVLink messages in a background thread, updates vehicle state
- LANDING_TARGET subscription is a topic (not service) because it's high-rate fire-and-forget at 20 Hz
- Mode switch and arm services block until ACK or timeout (2s), return success/failure
- All connection parameters (URI, baud, system ID, component ID) come from YAML config

**Future extensibility:** Adding new services (e.g. `send_servo_pwm`, `set_speed`, `send_waypoint`) requires only adding a new service server and the corresponding pymavlink call. No architectural changes needed.

### 3.2 Tag Detector Adapter Node (`tag_detector_adapter_node.py`)

Consumes raw AprilTag detections from whichever backend is active, applies dual-tag switching logic and debounce filtering, and publishes a single unified "best target" pose.

**Subscribed Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `/apriltag/detections` | `apriltag_msgs/AprilTagDetectionArray` | Raw detections from `apriltag_ros` or `isaac_ros_apriltag` |

**Published Topics:**

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/dbvf/landing_target_pose` | `LandingTargetPose` | Matches detection rate | Best tag pose: angles, position, tag_id, tag_size |
| `/dbvf/tag_status` | `TagStatus` | Matches detection rate | detected, active_tag_id, frames_since_last, confidence |

**Dual-tag switching logic:**

```
Tags:
  ID 0 - 0.6m  (primary, detected ~6m to ~1.5m altitude)
  ID 1 - 0.15m (secondary, detected ~1.5m to ground)

Rules:
  1. If only one tag detected -> use it
  2. If both detected -> use the larger tag (better pose estimate)
  3. If large tag lost and small tag detected -> switch after debounce
  4. If neither detected -> publish tag_status with detected=false,
     stop publishing landing_target_pose
```

**Debounce filter:**
- Maintains a rolling buffer of the last 30 frames (~1 second at 30 Hz)
- Tracks which tag ID was detected in each frame
- Only switches active tag when the new tag has 80%+ presence in the buffer (24/30 frames)
- Prevents rapid flip-flopping during the transition zone (~1.5m altitude)

**Pose output:**
- Computes `angle_x` and `angle_y` from tag center pixel offset and camera intrinsics (from `camera_info`)
- Passes through the 3D position from AprilTag pose estimation (body frame: x=forward, y=right, z=down)
- Includes both so the precision landing node can choose angles-only or full pose

**Configuration (from YAML):**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `primary_tag_id` | 0 | Large tag ID |
| `secondary_tag_id` | 1 | Small tag ID |
| `primary_tag_size` | 0.6 | Meters |
| `secondary_tag_size` | 0.15 | Meters |
| `debounce_buffer_size` | 30 | Frames |
| `debounce_threshold` | 0.8 | Required consistency (0-1) |
| `detection_topic` | `/apriltag/detections` | Remappable input topic |

**Boundary:** This node has no MAVLink communication and makes no flight control decisions. It reports what it sees; the landing node decides what to do with it.

### 3.3 Precision Landing Node (`precision_landing_node.py`)

State machine that orchestrates the full precision landing sequence.

**Subscribed Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `/dbvf/landing_target_pose` | `LandingTargetPose` | Best tag pose from adapter |
| `/dbvf/tag_status` | `TagStatus` | Detection status |
| `/dbvf/vehicle_state` | `VehicleState` | Current mode, altitude, position, velocity |

**Published Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `/dbvf/cmd/landing_target` | `LandingTargetPose` | Forwarded to mavlink_interface for MAVLink LANDING_TARGET |
| `/dbvf/landing_state` | `std_msgs/String` | Current state for monitoring/debugging |

**Trigger:** ROS2 service `/dbvf/start_precision_landing` with target GPS coordinates. The future mission FSM calls this service when it's time to land. Returns success/failure when the landing completes or aborts.

**State Machine:**

```
IDLE
  |  Called via service with target GPS coords
  v
APPROACH
  |  Switch to GUIDED mode
  |  Command position to target lat/lon at current altitude
  |  Wait until within position tolerance (default 2m lateral)
  |  Begin descent to approach_altitude (default 8m, matches PLND_ALT_MAX)
  |  Wait until altitude reached
  v
SEARCH
  |  Loiter at current position
  |  Descend slowly (default 0.3 m/s)
  |  Monitor /dbvf/tag_status for detection
  |  If tag detected -> DESCEND
  |  If altitude < min_search_altitude and no tag -> ABORT_LAND
  |  If timeout exceeded -> ABORT_LAND
  v
DESCEND
  |  Tag confirmed detected
  |  Forward /dbvf/landing_target_pose -> /dbvf/cmd/landing_target at 20 Hz
  |  Switch to LAND mode (ArduPilot PLND takes over lateral correction)
  |  Monitor tag_status: if tag lost for > tag_lost_timeout -> back to SEARCH
  |  Monitor vehicle_state: if landed detected -> LANDED
  v
LANDED
  |  Report success
  |  Return to IDLE (leave armed/disarmed based on caller preference)

ABORT_LAND
  |  Tag never found or search timeout
  |  Stay in LAND mode (GPS-only vertical descent)
  |  Report failure reason
  |  -> LANDED when touchdown detected
```

**Key behaviors:**

- **SEARCH -> DESCEND transition:** Requires `tag_status.detected == true` for `tag_confirm_frames` consecutive frames (default 5, ~0.5s) before committing. Prevents a single false positive from triggering LAND mode.
- **DESCEND -> SEARCH fallback:** If the tag is lost for longer than `tag_lost_timeout` (default 4s, matches `PLND_TIMEOUT`), switches back to GUIDED and re-enters SEARCH with continued slow descent.
- **Landing detection:** Monitors `vehicle_state` for disarmed status or altitude < 0.1m with near-zero velocity.
- **Control loop rate:** 20 Hz, matching the LANDING_TARGET send rate.

**Configuration (from YAML):**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `approach_altitude` | 8.0 | Altitude to begin search (meters, matches PLND_ALT_MAX) |
| `min_search_altitude` | 1.0 | Give up searching below this (meters) |
| `search_descent_rate` | 0.3 | Descent speed during search (m/s) |
| `position_tolerance` | 2.0 | Lateral tolerance for approach complete (meters) |
| `tag_confirm_frames` | 5 | Consecutive detections before committing |
| `tag_lost_timeout` | 4.0 | Seconds without tag before returning to SEARCH |
| `landing_timeout` | 60.0 | Total timeout for entire landing sequence (seconds) |

---

## 4. Message & Service Definitions

### Messages

**`LandingTargetPose.msg`:**
```
std_msgs/Header header
int32 tag_id
float64 tag_size
float64 angle_x
float64 angle_y
float64 position_x
float64 position_y
float64 position_z
bool position_valid
```

**`TagStatus.msg`:**
```
std_msgs/Header header
bool detected
int32 active_tag_id
int32 frames_since_last
float64 confidence
```

**`VehicleState.msg`:**
```
std_msgs/Header header
string mode
bool armed
float64 lat
float64 lon
float64 alt_rel
float64 vx
float64 vy
float64 vz
float64 heading
```

### Services

**`SetMode.srv`:**
```
string mode
---
bool success
string message
```

**`ArmMotors.srv`:**
```
bool arm
---
bool success
string message
```

**`SendGuidedPosition.srv`:**
```
float64 lat
float64 lon
float64 alt
---
bool success
string message
```

**`StartPrecisionLanding.srv`:**
```
float64 target_lat
float64 target_lon
---
bool success
string message
```

---

## 5. Launch Files & Platform Swap

### Sim Launch (`precision_landing_sim.launch.py`)

Launches:
1. `apriltag_ros` node — subscribes to `/camera/image` and `/camera/camera_info` from Gazebo bridge, publishes `/apriltag/detections`
2. `tag_detector_adapter_node` — subscribes to `/apriltag/detections`
3. `mavlink_interface_node` — loads `config/sim_params.yaml`, connects via TCP
4. `precision_landing_node` — loads `config/sim_params.yaml`

Requires the Gazebo sim (`iris_runway.launch.py`) to be running separately with the hardmount camera and AprilTags in the world.

### Jetson Launch (`precision_landing_jetson.launch.py`)

Launches:
1. `isaac_ros_apriltag` (+ `isaac_ros_argus_camera`, `rectify`) — publishes `/apriltag/detections` (same message type)
2. `tag_detector_adapter_node` — identical, no changes
3. `mavlink_interface_node` — loads `config/jetson_params.yaml`, connects via serial UART
4. `precision_landing_node` — identical, no changes

### Config Files

**`config/sim_params.yaml`:**
```yaml
mavlink_interface:
  ros__parameters:
    connection_string: "tcp:127.0.0.1:5760"
    source_system: 255
    source_component: 0
    heartbeat_rate: 1.0
    vehicle_state_rate: 10.0

tag_detector_adapter:
  ros__parameters:
    primary_tag_id: 0
    secondary_tag_id: 1
    primary_tag_size: 0.6
    secondary_tag_size: 0.15
    debounce_buffer_size: 30
    debounce_threshold: 0.8
    detection_topic: "/apriltag/detections"

precision_landing:
  ros__parameters:
    approach_altitude: 8.0
    min_search_altitude: 1.0
    search_descent_rate: 0.3
    position_tolerance: 2.0
    tag_confirm_frames: 5
    tag_lost_timeout: 4.0
    landing_timeout: 60.0
```

**`config/jetson_params.yaml`:**
```yaml
mavlink_interface:
  ros__parameters:
    connection_string: "/dev/ttyTHS1"
    baud_rate: 921600
    source_system: 255
    source_component: 0
    heartbeat_rate: 1.0
    vehicle_state_rate: 10.0

tag_detector_adapter:
  ros__parameters:
    primary_tag_id: 0
    secondary_tag_id: 1
    primary_tag_size: 0.6
    secondary_tag_size: 0.15
    debounce_buffer_size: 30
    debounce_threshold: 0.8
    detection_topic: "/apriltag/detections"

precision_landing:
  ros__parameters:
    approach_altitude: 8.0
    min_search_altitude: 1.0
    search_descent_rate: 0.3
    position_tolerance: 2.0
    tag_confirm_frames: 3
    tag_lost_timeout: 4.0
    landing_timeout: 60.0
```

---

## 6. Data Flow

```
[Camera Source]
  Sim: Gazebo bridge -> /camera/image + /camera/camera_info
  Jetson: isaac_ros_argus_camera -> NVMM GPU memory
    |
    v
[AprilTag Detector]
  Sim: apriltag_ros (CPU, ~6-12 fps)
  Jetson: isaac_ros_apriltag (CUDA, 120+ fps)
    |
    v
/apriltag/detections (AprilTagDetectionArray)
    |
    v
[tag_detector_adapter_node]
  Dual-tag switching + debounce
    |
    +---> /dbvf/landing_target_pose (LandingTargetPose)
    +---> /dbvf/tag_status (TagStatus)
    |
    v
[precision_landing_node]
  State machine: IDLE -> APPROACH -> SEARCH -> DESCEND -> LANDED
    |
    +---> /dbvf/cmd/landing_target (LandingTargetPose, 20 Hz)
    +---> /dbvf/landing_state (String)
    |     Calls services: /dbvf/set_mode, /dbvf/arm_motors,
    |                     /dbvf/send_guided_position
    v
[mavlink_interface_node]
  Owns pymavlink connection
    |
    +---> MAVLink LANDING_TARGET (20 Hz) -> ArduPilot
    +---> MAVLink SET_MODE, ARM, SET_POSITION_TARGET -> ArduPilot
    +---> MAVLink HEARTBEAT (1 Hz) -> ArduPilot
    |
    +<--- MAVLink HEARTBEAT, GLOBAL_POSITION_INT, etc. <- ArduPilot
    |
    +---> /dbvf/vehicle_state (VehicleState, 10 Hz)
    +---> /dbvf/heartbeat_status (Bool, 1 Hz)
```

---

## 7. ArduPilot Configuration

Add to `gazebo-iris-hardmount.parm`:

```
PLND_ENABLED    1
PLND_TYPE       1
PLND_EST_TYPE   0
PLND_ALT_MAX    8
PLND_ALT_MIN    0.5
PLND_STRICT     1
PLND_RET_MAX    3
PLND_TIMEOUT    4
PLND_ORIENT     25
PLND_LAG        0.05
```

---

## 8. Prerequisites

| Prerequisite | Status |
|-------------|--------|
| Hardmount camera model | Done |
| Camera bridge config | Done |
| AprilTag Gazebo models (ID 0, ID 1) | Done |
| AprilTags placed in iris_runway.sdf | Done |
| PLND parameters in parm file | Needed |
| `apriltag_ros` (christianrauch) installed | Needed |
| `pymavlink` pip installed | Needed |
| `dbvf_autonomy` package created | Needed |

---

## 9. Future Extensibility

The `dbvf_autonomy` package is designed to grow beyond precision landing:

- **Mission FSM node:** Calls `/dbvf/start_precision_landing` service, orchestrates full competition sequence. Uses `mavlink_interface` services for mode switches, waypoint commands.
- **Payload pickup node:** Post-landing mechanism control via new `send_servo_pwm` service on `mavlink_interface`.
- **Drop zone detection node:** Consumes camera feed, publishes zone poses. Uses `mavlink_interface` for GUIDED velocity commands.
- **Payload drop node:** Visual servoing + servo actuation via `mavlink_interface`.

All future nodes use the same `mavlink_interface` — no additional MAVLink connections needed.
