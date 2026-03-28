# Precision Landing System — Implementation Log

**Date:** 2026-03-27
**Plan:** `docs/superpowers/plans/2026-03-27-precision-landing.md`
**Design Spec:** `docs/superpowers/specs/2026-03-27-precision-landing-design.md`
**Method:** Subagent-Driven Development (Opus 4.6 for all roles)

---

## Summary

Built a complete ROS2 AprilTag-based precision landing system across 10 tasks. The system consists of three custom nodes (MAVLink interface, tag detector adapter, precision landing state machine), custom messages/services, and a simulation launch file. All 38 unit tests pass.

---

## Task Execution Log

### Task 1: Package Scaffolding
**Status:** Complete | **Commit:** `c603801`

- Created `src/dbvf_autonomy/` with CMakeLists.txt, package.xml, setup.py, resource marker, `__init__.py`
- **Deviation:** Changed `<build_type>` from `ament_cmake_python` to `ament_cmake` in package.xml. Reason: colcon 0.5.0 does not have a build extension for `ros.ament_cmake_python`, causing the package to be skipped. Using `ament_cmake` with `find_package(ament_cmake_python REQUIRED)` in CMakeLists.txt is the standard pattern for mixed packages with rosidl interfaces.
- Spec review: Compliant (deviation justified)
- Code quality review: Noted forward-declared files cause build failure — by-design per plan, resolved in subsequent tasks

### Task 2: Message and Service Definitions
**Status:** Complete | **Commit:** `90cda1b`

- Created 3 messages (LandingTargetPose, TagStatus, VehicleState) and 4 services (SetMode, ArmMotors, SendGuidedPosition, StartPrecisionLanding)
- **Major deviation:** Split messages into separate `dbvf_msgs` package (ament_cmake). Reason: namespace collision between rosidl-generated Python code and the `dbvf_autonomy` Python source package — `rosidl_generate_interfaces` and `ament_python_install_package` cannot coexist with the same package name. The plan anticipated this possibility: *"If the Python import fails due to namespace collision, split messages into a separate `dbvf_msgs` package."*
- **Impact on subsequent tasks:** All imports changed from `dbvf_autonomy.msg/srv` to `dbvf_msgs.msg/srv`
- New package: `src/dbvf_msgs/` with its own CMakeLists.txt and package.xml
- dbvf_autonomy CMakeLists.txt updated: removed `rosidl_generate_interfaces` block, added `dbvf_msgs` dependency
- Verified: `from dbvf_msgs.msg import LandingTargetPose, TagStatus, VehicleState` works
- Verified: `from dbvf_msgs.srv import SetMode, ArmMotors, SendGuidedPosition, StartPrecisionLanding` works
- Spec review: Compliant — all fields match exactly

### Task 3: Install apriltag_ros
**Status:** Complete | **Commit:** `4bd1a3e`

- Installed apriltag C library from source to `~/.local` (sudo unavailable for apt)
- Cloned `apriltag_msgs` (v2.0.1) and `apriltag_ros` (v3.3.0) into `src/`
- **Note:** apriltag_ros `master` branch is incompatible with ROS2 Humble (uses `.hpp` headers not present in Humble's tf2_ros). Tag `3.3.0` is the correct version.
- **Note:** Future builds may need `CMAKE_PREFIX_PATH="/home/finn/.local"` for the apriltag C library
- Verified: `ros2 pkg executables apriltag_ros` returns `apriltag_ros apriltag_node`
- Created workspace-level `.gitignore` and `dependencies.repos` for reproducible setup
- No spec/code review needed (dependency installation, no custom code)

### Task 4: ArduPilot PLND Parameters
**Status:** Complete | **Committed in ardupilot_gazebo repo**

- Appended 10 PLND parameters to `src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm`:
  - `PLND_ENABLED 1`, `PLND_TYPE 1`, `PLND_EST_TYPE 0`, `PLND_ALT_MAX 8`, `PLND_ALT_MIN 0.5`
  - `PLND_STRICT 1`, `PLND_RET_MAX 3`, `PLND_TIMEOUT 4`, `PLND_ORIENT 25`, `PLND_LAG 0.05`
- Rebuilt ardupilot_gazebo package
- No spec/code review needed (configuration only)

### Task 5: Sim Configuration File
**Status:** Complete | **Commit:** `1695b8d`

- Created `src/dbvf_autonomy/config/sim_params.yaml` with parameters for all 3 nodes:
  - `mavlink_interface`: connection_string, source_system, source_component, heartbeat_rate, vehicle_state_rate
  - `tag_detector_adapter`: primary/secondary tag IDs and sizes, debounce settings, detection topic
  - `precision_landing`: approach_altitude, min_search_altitude, search_descent_rate, position_tolerance, tag_confirm_frames, tag_lost_timeout, landing_timeout
- Verified: config installed to `install/dbvf_autonomy/share/dbvf_autonomy/config/sim_params.yaml`
- No spec/code review needed (YAML configuration)

### Task 6: MAVLink Interface Node
**Status:** Complete | **Commit:** `7907826`

- **TDD:** Wrote `test/test_mavlink_messages.py` first (3 tests), verified failure, then implemented
- Created `dbvf_autonomy/mavlink_interface_node.py` (275 lines):
  - `ARDUPILOT_MODE_MAP` — 22 ArduCopter mode mappings
  - `build_landing_target_params()` — pure function for MAVLink LANDING_TARGET construction
  - `MavlinkInterfaceNode` — ROS2 node owning the pymavlink TCP connection
    - Publishers: `/dbvf/vehicle_state` (10Hz), `/dbvf/heartbeat_status` (1Hz)
    - Subscriber: `/dbvf/cmd/landing_target` (forwarded as MAVLink LANDING_TARGET)
    - Services: `/dbvf/set_mode`, `/dbvf/arm_motors`, `/dbvf/send_guided_position`
    - Auto-reconnection on connection loss
- Created `scripts/mavlink_interface_node` entry point
- Tests: 3/3 passing
- Spec review: Compliant — all 22 modes, all topics/services, correct imports
- Code quality review: Noted thread safety patterns (safe with SingleThreadedExecutor), missing chmod (sandbox limitation, handled by CMake install)

### Task 7: Tag Detector Adapter Node
**Status:** Complete | **Commit:** `50cf428`

- **TDD:** Wrote 3 test files first (17 tests total), verified failures, then implemented
- Created `dbvf_autonomy/tag_detector_adapter_node.py`:
  - `DebounceFilter` — rolling-buffer debounce (configurable buffer_size and threshold)
  - `compute_angles(u, v, cx, cy, fx, fy)` — atan2-based pixel-to-angle conversion
  - `select_best_tag()` — primary tag preferred, fallback to secondary
  - `TagDetectorAdapterNode` — ROS2 node
    - Subscribes: `/apriltag/detections` (AprilTagDetectionArray), `/camera/camera_info` (CameraInfo)
    - Publishes: `/dbvf/landing_target_pose` (LandingTargetPose), `/dbvf/tag_status` (TagStatus)
    - 7 configurable parameters
- Created `scripts/tag_detector_adapter_node` entry point
- Tests: 17/17 passing (7 debounce + 5 angle + 5 tag selection)
- Spec review: Compliant
- Code quality review: Suggested deque over list (negligible at buffer_size=30) and module extraction (would break plan structure)

### Task 8: Precision Landing Node
**Status:** Complete | **Commit:** `b17bfda`

- **TDD:** Wrote `test/test_state_machine.py` first (13 tests), verified failure, then implemented
- Created `dbvf_autonomy/precision_landing_node.py`:
  - `LandingState` enum — IDLE, APPROACH, SEARCH, DESCEND, LANDED, ABORT_LAND
  - `LandingStateMachine` — pure Python FSM
    - APPROACH: fly to target GPS in GUIDED mode, transition when within position_tolerance
    - SEARCH: slow descent (0.3 m/s), require tag_confirm_frames consecutive detections
    - DESCEND: LAND mode, forward LANDING_TARGET at 20Hz, tag_lost_timeout fallback
    - LANDED: detected via disarm or alt<0.1m + |vz|<0.1
    - ABORT_LAND: GPS-only LAND mode on min altitude or global timeout
  - `PrecisionLandingNode` — ROS2 node with 20Hz control loop
    - Subscribers: `/dbvf/landing_target_pose`, `/dbvf/tag_status`, `/dbvf/vehicle_state`
    - Publishers: `/dbvf/cmd/landing_target`, `/dbvf/landing_state`
    - Service clients: `/dbvf/set_mode`, `/dbvf/send_guided_position`
    - Service server: `/dbvf/start_precision_landing`
- Created `scripts/precision_landing_node` entry point
- Tests: 13/13 FSM tests passing; **38/38 total across full suite**
- Spec review: Compliant — all 6 states, all transitions, all topics/services/parameters match
- Note: Plan mentioned "14 tests" but only 13 test functions were specified in the plan text

### Task 9: Sim Launch File
**Status:** Complete | **Commit:** `8a16f13`

- Created `launch/precision_landing_sim.launch.py` wiring 4 nodes:
  1. `apriltag_ros/apriltag_node` with camera remappings and tag36h11 family config
  2. `dbvf_autonomy/tag_detector_adapter_node` with sim_params.yaml
  3. `dbvf_autonomy/mavlink_interface_node` with sim_params.yaml
  4. `dbvf_autonomy/precision_landing_node` with sim_params.yaml
- Verified: `ros2 launch dbvf_autonomy precision_landing_sim.launch.py --show-args` parses successfully

### Task 10: Integration Smoke Test
**Status:** Complete (2026-03-27)

Tested precision landing in Gazebo sim. Resolved 4 integration issues (ISS-001 through ISS-004) before achieving successful landing sequence.

**Issues encountered and resolved:**
- **ISS-001:** `libapriltag.so.3` not found — copied v3.4.5 library to `/usr/local/lib/` with proper symlinks
- **ISS-002:** `mavlink_interface` connected with `sysid=0 compid=0` — changed connection from TCP:5760 to UDP:14551 (MAVProxy relay output)
- **ISS-003:** Mode switch to GUIDED failed — resolved by ISS-002 fix (correct sysid/compid)
- **ISS-004:** Drone navigated to wrong location — three compounding causes:
  1. **(Primary)** Swapped ENU→GPS coordinate conversion: Gazebo X=50m (East) was applied as latitude offset instead of longitude offset, and Y=1m (North) as longitude instead of latitude. Correct tag GPS: `lat=-35.3632531, lon=149.1657896`
  2. MAVProxy relay stripped `compid` to 0 — fixed by forcing `target_component=1` after `wait_heartbeat()`
  3. APPROACH state sent guided position only once — added periodic re-send every 0.5s (matching SEARCH pattern)

**Code fixes applied:**
- `mavlink_interface_node.py`: Force `self.conn.target_component = 1` after `wait_heartbeat()` (MAVProxy relay reports compid=0)
- `precision_landing_node.py`: Added periodic guided position re-send in APPROACH state control loop (every 0.5s)
- `sim_params.yaml`: Connection string changed from `tcp:127.0.0.1:5760` to `udpin:0.0.0.0:14551`

**Successful test procedure:**
1. **Terminal 1:** `ros2 launch ardupilot_gz_bringup iris_runway.launch.py rviz:=true use_gz_tf:=true`
2. **Terminal 2:** `ros2 launch dbvf_autonomy precision_landing_sim.launch.py`
3. **MAVProxy:** `mavproxy.py --master udpin:0.0.0.0:14550 --console` → `mode guided` → `arm throttle` → `takeoff 10`
4. **Trigger:** `ros2 service call /dbvf/start_precision_landing dbvf_msgs/srv/StartPrecisionLanding "{target_lat: -35.3632531, target_lon: 149.1657896}"`
5. **Result:** APPROACH → SEARCH → DESCEND → LANDED sequence completed successfully

**ENU→GPS coordinate conversion reference:**
- World GPS origin: `lat=-35.3632621, lon=149.1652374` (from `iris_runway.sdf`)
- Tag Gazebo pose: `(50.0, 1.0, 2.457)` = 50m East, 1m North
- Formula: `lat = origin_lat + Y_north/111000`, `lon = origin_lon + X_east/(111000 × cos(origin_lat))`
- Tag GPS: `lat=-35.3632531, lon=149.1657896`

---

## Final State

### Packages
| Package | Type | Purpose |
|---------|------|---------|
| `dbvf_msgs` | ament_cmake | Message and service definitions (3 msgs, 4 srvs) |
| `dbvf_autonomy` | ament_cmake (with ament_cmake_python) | Python nodes, config, launch |

### Test Results
```
38 tests, 0 errors, 0 failures, 0 skipped
```
- test_mavlink_messages: 3 tests (mode map, landing target params)
- test_debounce_filter: 7 tests (rolling buffer debounce logic)
- test_angle_computation: 5 tests (pixel-to-angle conversion)
- test_tag_selection: 5 tests (primary/secondary tag priority)
- test_state_machine: 13 tests (all FSM state transitions)

Requires: `source /opt/ros/humble/setup.bash && source install/setup.bash` before running `colcon test`.

### Git History (src/dbvf_autonomy)
```
8a16f13 feat: add sim launch file wiring apriltag_ros and landing nodes
b17bfda feat: add precision landing state machine node
50cf428 feat: add tag detector adapter with debounce and dual-tag switching
7907826 feat: add mavlink interface node with pymavlink connection and services
1695b8d feat: add sim parameter config for precision landing nodes
4bd1a3e chore: add apriltag_ros and apriltag_msgs as build dependencies
90cda1b feat: add message and service definitions for precision landing
c603801 feat: scaffold dbvf_autonomy package with ament_cmake_python
```

### ISS-006 Fix: AprilTag Occlusion (2026-03-28)

**Problem:** Primary tag (ID 0, 0.6m) never detected at any altitude. Secondary tag (ID 1, 0.15m) stacked directly on top at same XY, 1mm higher — occluding centre data bits of the primary tag pattern.

**Root cause confirmed by investigation:**
- `iris_runway.sdf`: both tags at XY (50.0, 1.0), Z separated by 1mm
- Secondary tag (0.15m) covers ±0.075m from centre of primary tag's 0.36m data region (~42% of data width)
- apriltag_ros config verified correct (tag.ids/sizes paired by index, size only affects pose estimation)
- Primary tag pixel size at 8m = 15.4px (sufficient for detection if unoccluded)

**Fix applied:** Offset secondary tag 0.5m in X (East) in `iris_runway.sdf`:
- Primary: `(50.0, 1.0, 2.457)` — unchanged
- Secondary: `(50.5, 1.0, 2.457)` — offset 0.5m East, same Z (flush)
- Edge-to-edge gap = 0.125m; both tags in camera FOV at all descent altitudes (HFOV ≈ 114.5°)

**File changed:** `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf`
**Rebuild required:** `colcon build --packages-select ardupilot_gz_gazebo`

---

### Known Issues / Notes
1. **Script permissions:** Entry point scripts in `scripts/` lack `+x` bit in source tree (sandbox blocked chmod). Not a problem — CMake `install(PROGRAMS ...)` sets executable permissions at install time.
2. **apriltag C library:** v3.4.5 installed to both `~/.local` and `/usr/local/lib/` (ISS-001 resolved). May need `CMAKE_PREFIX_PATH="/home/finn/.local"` for clean rebuilds.
3. **apriltag_ros version:** Must use tag `3.3.0` (not master) for ROS2 Humble compatibility.
4. **position_valid:** Currently `False` in tag_detector_adapter — position estimation deferred until camera-to-body frame transform is verified in sim.
5. **Angle sign tuning:** If drone moves away from tag during DESCEND, `angle_x`/`angle_y` signs in `compute_angles()` may need to be negated. This is the most likely tuning issue during integration testing.
6. **ENU→GPS conversion gotcha:** Gazebo X=East→longitude, Y=North→latitude. Easy to swap. Reference formula in Task 10 notes.
7. **MAVProxy compid relay:** MAVProxy UDP relay reports `compid=0`. `mavlink_interface_node` forces `target_component=1` after `wait_heartbeat()` to work around this.
