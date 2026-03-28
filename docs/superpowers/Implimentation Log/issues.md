# Issue Tracker

---

## ISS-001: apriltag_node crashes — libapriltag.so.3 not found

**Status:** Closed
**Date:** 2026-03-27
**Severity:** Blocking
**Component:** apriltag_ros / apriltag C library
**Resolved:** 2026-03-27

### Problem

`apriltag_node` dies immediately on launch with:
```
Could not load library dlopen error: libapriltag.so.3: cannot open shared object file: No such file or directory
```

The apriltag C library was installed to `~/.local` during Task 3 (sudo was unavailable for system-wide install). The shared library `libapriltag.so.3` exists there but is not on the runtime library search path.

### Resolution

1. Installed `libapriltag-dev` (v3.2.0) via apt — but this version was missing `estimate_pose_for_tag_homography` symbol needed by apriltag_ros v3.3.0.
2. Copied the locally-built v3.4.5 library to `/usr/local/lib/` with proper symlinks:
   ```bash
   sudo cp ~/.local/lib/libapriltag.so.3.4.5 /usr/local/lib/
   sudo ln -sf /usr/local/lib/libapriltag.so.3.4.5 /usr/local/lib/libapriltag.so.3
   sudo ln -sf /usr/local/lib/libapriltag.so.3.4.5 /usr/local/lib/libapriltag.so
   sudo ldconfig
   ```
3. Verified `ldconfig -p | grep apriltag` shows `/usr/local/lib/libapriltag.so.3` (v3.4.5 takes priority).
4. Verified `apriltag_node` loads and runs without dlopen errors.

---

## ISS-002: mavlink_interface connects with sysid=0 compid=0

**Status:** Closed
**Date:** 2026-03-27
**Severity:** Blocking
**Component:** mavlink_interface_node
**Resolved:** 2026-03-27

### Problem

`mavlink_interface_node` reports `Connected: sysid=0 compid=0` instead of the expected `sysid=1 compid=1`. This causes all subsequent MAVLink commands (mode switches, guided position, etc.) to be sent to system 0 / component 0, which ArduPilot ignores.

### Root Cause

The bringup launch (`iris_runway.launch.py`) already starts a non-interactive MAVProxy on `tcp:127.0.0.1:5760` with `--out 127.0.0.1:14550 --out 127.0.0.1:14551`. When `mavlink_interface_node` also tried to connect to `tcp:5760`, SITL's single-TCP-client limit caused it to get a degraded connection with `sysid=0 compid=0`.

### Resolution

Used **Option A (MAVProxy as multiplexer)** — the bringup MAVProxy already multiplexes to UDP outputs:
1. Changed `sim_params.yaml` connection_string from `tcp:127.0.0.1:5760` to `udpin:0.0.0.0:14551`.
2. Changed default in `mavlink_interface_node.py` to match.
3. Updated docs: manual MAVProxy console now uses `mavproxy.py --master udpin:0.0.0.0:14550 --console`.

The bringup MAVProxy forwards bidirectionally — commands sent via `udpin` connections are relayed to SITL through MAVProxy.

---

## ISS-003: Mode switch to GUIDED fails — precision landing cannot start

**Status:** Closed
**Date:** 2026-03-27
**Severity:** Blocking
**Component:** precision_landing_node / mavlink_interface_node
**Depends on:** ISS-002
**Resolved:** 2026-03-27

### Problem

After triggering `/dbvf/start_precision_landing`, the precision landing node attempts to switch to GUIDED mode but gets:
```
Mode: Mode switch to GUIDED failed
```

### Root Cause

Direct consequence of ISS-002. Because `mavlink_interface_node` had `target_system=0` and `target_component=0`, the `MAV_CMD_DO_SET_MODE` command was addressed to the wrong target.

### Resolution

Resolved by ISS-002 fix. With the connection routed through the bringup MAVProxy's UDP output, `mavlink_interface_node` receives proper heartbeats with `sysid=1 compid=1`, and mode switch commands are correctly addressed.

---

## ISS-004: Drone navigates to wrong point during precision landing approach

**Status:** Closed
**Date:** 2026-03-27
**Severity:** Blocking
**Component:** precision_landing_node / mavlink_interface_node
**Depends on:** ISS-002 (resolved)
**Resolved:** 2026-03-27

### Problem

After triggering `/dbvf/start_precision_landing` with correct tag coordinates (`target_lat: -35.3628114, target_lon: 149.1652484`), the drone enters GUIDED mode and moves, but navigates to the wrong location instead of the target position.

### Observations

- `mavlink_interface_node` logs: `Connected: sysid=1 compid=0` — **compid is 0, should be 1** (MAV_COMP_ID_AUTOPILOT1). The bringup MAVProxy relay may be stripping or modifying the component ID from the heartbeat.
- The drone moves to a **completely different location** (not a small offset) and lands there.
- ISS-002 port fix is partially effective (sysid=1 is correct, connection established), but compid=0 means commands are addressed to component 0 instead of the autopilot.

### Root Cause

Three compounding issues:

1. **Swapped ENU→GPS coordinate conversion (primary).** The target coordinates passed to the service had the Gazebo X (East) and Y (North) offsets applied to the wrong GPS axes. The tag is at Gazebo pose `(50.0, 1.0)` = 50m East, 1m North. But the coordinates `-35.3628114, 149.1652484` encode 50m in latitude (North) and 1m in longitude (East) — exactly backwards. Correct coordinates: `lat=-35.3632531, lon=149.1657896`.

2. **compid=0 routing issue.** `wait_heartbeat()` picks up the MAVProxy-relayed heartbeat which has `compid=0`. All `command_long_send` and `set_position_target_global_int_send` calls used `self.conn.target_component` (= 0). Commands addressed to component 0 may be handled differently by ArduPilot or misrouted by MAVProxy.

3. **Single-shot APPROACH command.** The APPROACH state only sent the guided position command once (in `_start_landing_cb`). If that command was dropped over UDP, the drone flew to its previous GUIDED target.

### Resolution

1. **Correct coordinates** — Recalculated from world file: origin `(-35.3632621, 149.1652374)`, tag at Gazebo `(50.0 East, 1.0 North)` → GPS `lat=-35.3632531, lon=149.1657896`. Conversion: lat offset = Y_north / 111000, lon offset = X_east / (111000 × cos(lat)).

2. **Force `target_component=1`** — In `mavlink_interface_node.py`, added `self.conn.target_component = 1` immediately after `wait_heartbeat()`. ArduPilot's autopilot is always MAV_COMP_ID_AUTOPILOT1 (=1).

3. **Periodic re-send in APPROACH** — Added continuous guided position re-send every 0.5s in the APPROACH state, matching the existing SEARCH state pattern.

All 38 existing unit tests pass with no regressions.

---

## ISS-005: SEARCH state descends without tag detection; no visual debug feedback

**Status:** Open
**Date:** 2026-03-27
**Severity:** Enhancement
**Component:** precision_landing_node / new tag_visualizer_node

### Problem

Two issues hindering integration testing:

1. **Blind descent in SEARCH:** The SEARCH state descends at 0.3 m/s continuously regardless of whether the AprilTag is detected. The drone reaches `min_search_altitude` and aborts before tag confirmation can occur. For testing precision landing, the drone should hold altitude until it sees the tag, then descend.

2. **No visual feedback of tag detections:** There is no way to see what the camera detects. apriltag_ros v3.3.0 does not publish annotated debug images. An OpenCV overlay showing detected tag outlines (similar to previous simulation) is needed.

### Plan

1. **Hold altitude in SEARCH until tag detected** — Only decrease `_search_target_alt` when `latest_tag_status.detected` is True. Drone holds at approach altitude while scanning, descends only when tag is in view.

2. **Tag visualizer node** — New `tag_visualizer_node.py` subscribing to `/camera/image` and `/apriltag/detections`, drawing tag outlines and IDs via OpenCV, publishing to `/dbvf/debug/tag_image`. Viewable with `rqt_image_view`.

---

## ISS-006: Primary AprilTag (ID 0, 0.6m) not detected at approach altitude

**Status:** Fixed — awaiting integration retest
**Date:** 2026-03-28
**Severity:** Blocking
**Component:** Gazebo world layout (`iris_runway.sdf`)

### Problem

When the drone arrives at the correct GPS coordinates and hovers over the AprilTag at ~8m altitude, the primary tag (ID 0, 0.6m) is **not detected** even though it is clearly visible in the camera feed. Only the smaller secondary tag (ID 1, 0.15m) is detected, and only when the drone is less than ~1m above the pad.

The tag visualizer confirms the tag is in frame and visually recognizable, but apriltag_ros does not report a detection for ID 0 at higher altitudes.

### Root Cause — CONFIRMED: Secondary tag occludes primary tag centre

**Evidence from `iris_runway.sdf:115-126`:**
```xml
<include>
  <name>apriltag_wa_primary</name>      <!-- ID 0, 0.6m -->
  <uri>model://Apriltag36_11_00000</uri>
  <pose>50.0 1.0 2.457 0 0 0</pose>
</include>
<include>
  <name>apriltag_wa_secondary</name>    <!-- ID 1, 0.15m -->
  <uri>model://Apriltag36_11_00001</uri>
  <pose>50.0 1.0 2.458 0 0 0</pose>     <!-- same XY, 1mm higher -->
</include>
```

Both tags share **identical XY** (50.0, 1.0). The secondary sits **1mm above** the primary (Z 2.458 vs 2.457). Gazebo renders the higher surface on top.

**Why this prevents detection:** tag36h11 layout is 10×10 cells (1-cell white border, 1-cell black border, 6×6 data payload). On the 0.6m tag, each cell = 0.06m. The data region spans the centre 0.36m (±0.18m from centre). The 0.15m secondary tag covers ±0.075m from centre — **it occludes ~42% of the data bit width**, corrupting the centre of the data pattern. The apriltag detector cannot decode a tag with corrupted centre data at any altitude.

In the real world these are separate printed sheets placed side-by-side — not stacked.

### Ruled Out

| Hypothesis | Finding |
|------------|---------|
| **apriltag_ros config wrong** | Config is correct. `tag.ids: [0, 1]` and `tag.sizes: [0.6, 0.15]` are paired by index in `AprilTagNode.cpp:153-159`. Default `size: 0.6` only applies to tags NOT in `tag.ids`. |
| **Tag too small at 8m** | Primary tag at 8m = 15.4px across (1.54 px/cell), data grid = 9.2px. Above minimum (~6px data grid). Would be detectable if not occluded. |
| **apriltag_ros detection thresholds** | Size parameter only affects pose estimation, not detection (`AprilTagNode.cpp:242`). Detection is purely image-based. |

### Pixel Size Analysis (camera: fx=205.5, 640×480)

| Tag | Size | Alt | Pixels | px/cell | Detectable? |
|-----|------|-----|--------|---------|-------------|
| ID 0 | 0.6m | 8m | 15.4 | 1.54 | Yes (if unoccluded) |
| ID 0 | 0.6m | 4m | 30.8 | 3.08 | Yes (solid) |
| ID 1 | 0.15m | 8m | 3.9 | 0.39 | **No** (< 1 px/cell) |
| ID 1 | 0.15m | 2m | 15.4 | 1.54 | Marginal |
| ID 1 | 0.15m | 1m | 30.8 | 3.08 | Yes |

Formula: `pixels = (tag_size × fx) / altitude`

Secondary tag undetectable above ~2m — consistent with observed "only below ~1m" behaviour.

### Fix Plan

Offset secondary tag 0.5m in X (East) in `iris_runway.sdf`:

```xml
<!-- Primary — unchanged -->
<pose>50.0 1.0 2.457 0 0 0</pose>

<!-- Secondary — offset 0.5m East, same Z (flush, no z-fighting) -->
<pose>50.5 1.0 2.457 0 0 0</pose>
```

**Why 0.5m offset:**
- Edge-to-edge gap: 0.5 − 0.3 − 0.075 = 0.125m (clear separation)
- Camera HFOV ≈ 114.5° → ground coverage at 0.5m alt is ~1.6m wide — both tags in frame at all descent altitudes
- Same Z avoids z-fighting artefacts

After fix, rebuild `ardupilot_gz_gazebo` and re-test detection at 8m.

---

## ISS-007: Hard landing — no hover/hold phase for secondary tag acquisition

**Status:** Open
**Date:** 2026-03-28
**Severity:** Major
**Component:** precision_landing_node / mavlink_interface_node

### Problem

The drone descends too quickly during the final approach and lands hard. The current DESCEND state hands off to ArduPilot PLND in LAND mode, which controls descent rate autonomously — there is no intermediate phase where the drone holds altitude, acquires the secondary tag (ID 1, 0.15m), and performs fine XY positioning before touching down.

In the real-world competition, the drone must position itself precisely over a **payload reloading mechanism** using the secondary tag. This requires:
1. Stopping descent at a hold altitude (~1m AGL, tuneable based on secondary tag detection range)
2. Switching from primary tag (coarse guidance) to secondary tag (fine positioning)
3. Performing XY corrections relative to the secondary tag to align with the payload mechanism
4. Only then completing the final descent to make contact

### Context

From ISS-006 pixel analysis, the secondary tag (0.15m) becomes reliably detectable at ~1m altitude (30.8px, 3.08 px/cell) and marginally at ~2m (15.4px, 1.54 px/cell). The hold altitude threshold should be set where the secondary tag is reliably detected — approximately **1–1.5m AGL**.

The XY offset between the secondary tag and the physical payload mechanism will need to be calibrated on the real hardware and made configurable.

### Proposed Approach

1. **Lidar rangefinder integration** — Add a lidar/rangefinder altitude source (more accurate than GPS/baro near ground) to determine precise AGL height. The mavlink_interface_node or a new sensor node would publish rangefinder data.

2. **New FSM state: FINE_POSITION** — Insert between DESCEND and LANDED:
   - **Entry condition:** Rangefinder altitude ≤ hold threshold (configurable, default ~1m)
   - **Behaviour:** Switch to GUIDED or LOITER mode, hold altitude, wait for secondary tag (ID 1) detection
   - **XY control:** Use secondary tag angular offsets to send GUIDED position corrections, centering the drone over the tag (and eventually over the payload mechanism with a configurable XY offset)
   - **Exit condition:** XY error within tolerance for N consecutive frames → final slow descent to contact
   - **Timeout:** If secondary tag not acquired within timeout → fall back to LAND at current position

3. **Configurable parameters:**
   - `fine_position_altitude` — rangefinder threshold to enter FINE_POSITION (default 1.0m)
   - `fine_position_xy_tolerance` — acceptable XY error before final descent (metres)
   - `fine_position_offset_x/y` — offset from secondary tag to payload mechanism centre (0.0 initially, tuned on hardware)
   - `fine_descent_rate` — final descent speed after alignment (much slower than current)

4. **Physical calibration (future)** — The XY offset between secondary tag and payload mechanism needs to be measured on the real drone/pad and set via parameters. This is hardware-dependent and will be tuned during field testing.

### Dependencies

- Lidar rangefinder sensor must be added to Gazebo model and bridged to ROS2
- Secondary tag detection must work reliably at hold altitude (depends on ISS-006 fix)
- May require changes to `tag_detector_adapter_node` to report which tag ID is currently detected with higher granularity

---

## ISS-008: ArduPilot PreArm failure — Rangefinder 1: No Data

**Status:** Fixed — awaiting integration retest
**Date:** 2026-03-28
**Severity:** Blocking
**Component:** ArduPilot SITL / ArduPilotPlugin rangefinder config
**Resolved:** 2026-03-28

### Problem

After implementing the rangefinder integration (RNGFND1 parameters + gpu_lidar sensor + ArduPilotPlugin sensor block), the drone fails to arm with:

```
Got COMMAND_ACK: COMPONENT_ARM_DISARM: FAILED
AP: Arm: Rangefinder 1: No Data
AP: PreArm: Rangefinder 1: No Data
```

ArduPilot SITL is configured with `RNGFND1_TYPE 100` (SITL backend) and expects rangefinder data via the `rng_1` field in the JSON state packet from the ArduPilotPlugin. The PreArm check indicates that no rangefinder data is reaching ArduPilot despite the Gazebo sensor and plugin sensor block being configured.

### Root Cause — ArduPilotPlugin sensor block added to wrong model file

The rangefinder `<sensor>` block was added to `iris_with_gimbal/model.sdf`, but the simulation actually uses `iris_with_hardmount_camera/model.sdf` (included as `model://iris_with_hardmount_camera` in `ardupilot_gz_gazebo/worlds/iris_runway.sdf`).

**Data flow analysis:**

| Component | File | Status |
|-----------|------|--------|
| gpu_lidar sensor on `rangefinder_link` | `iris_with_standoffs/model.sdf` | Correct — merged into runtime model via `<include merge="true">`, publishes to `/rangefinder` |
| RNGFND1 parameters | `gazebo-iris-hardmount.parm` | Correct — `TYPE 100`, `ORIENT 25`, min/max match sensor |
| ROS2-Gazebo bridge | `iris_bridge.yaml` | Correct — maps `/rangefinder` GZ_TO_ROS |
| ArduPilotPlugin `<sensor>` block | `iris_with_gimbal/model.sdf` | **Wrong file** — this model is NOT used at runtime |
| ArduPilotPlugin (runtime) | `iris_with_hardmount_camera/model.sdf` | **Missing** `<sensor>` block — no subscription to `/rangefinder`, no `rng_1` in JSON state to SITL |

The plugin source (`ArduPilotPlugin.cc:879-992`) confirms that `LoadRangeSensors()` reads `<sensor>` elements from the plugin SDF and subscribes directly to the specified `<topic>`. Without the `<sensor>` block, the plugin never subscribes, never calls `RangeCb()`, and the `ranges[]` vector stays empty — so `CreateStateJSON()` never includes `rng_1` in the JSON state packet.

### Ruled Out

| Hypothesis | Finding |
|------------|---------|
| **Topic mismatch (relative vs absolute)** | Confirmed working via `iris_with_lidar` example: sensor `<topic>lidar</topic>` (relative) publishes to `/lidar`, bridge receives on `/lidar`. gz-transport normalizes both to the same topic. |
| **Sensor not spawning** | Sensor is on `rangefinder_link` in `iris_with_standoffs`, correctly merged into `iris_with_hardmount_camera` via `<include merge="true">`. The sensor exists in the runtime model. |
| **SITL parameter issue** | `RNGFND1_TYPE 100` is correct for SITL backend. The issue is no data arriving, not a parameter misconfiguration. |

### Resolution

Added the `<sensor>` block to the ArduPilotPlugin in `iris_with_hardmount_camera/model.sdf`, after `<imuName>imu_link::imu_sensor</imuName>`:

```xml
<sensor>
  <type>lidar</type>
  <index>1</index>
  <topic>/rangefinder</topic>
</sensor>
```

Rebuilt `ardupilot_gazebo`. All 66 unit tests pass. Integration retest needed to confirm the drone arms and rangefinder data flows end-to-end.

### Verification Steps

1. Launch Gazebo + SITL: `ros2 launch ardupilot_gz_bringup iris_runway.launch.py`
2. Check Gazebo topic: `gz topic -l | grep rangefinder` (should show `/rangefinder`)
3. Check ArduPilotPlugin log for: `[iris] subscribing to /rangefinder`
4. Arm via MAVProxy: `mode guided` → `arm throttle` (should succeed without PreArm failure)
5. Verify `rng_1` flows: `ros2 topic echo /dbvf/vehicle_state --field range_alt`

---

## ISS-009: SEARCH state holds altitude too high for tag detection — no step-down

**Status:** Open
**Date:** 2026-03-28
**Severity:** Major
**Component:** precision_landing_node (SEARCH state)

### Problem

When triggering precision landing with:
```bash
ros2 service call /dbvf/start_precision_landing dbvf_msgs/srv/StartPrecisionLanding \
  "{target_lat: -35.3632531, target_lon: 149.1657896}"
```

The drone approaches at `approach_altitude` (8.0m), transitions to SEARCH, and then **holds altitude indefinitely** because the AprilTag is not detectable at 8m. From ISS-006 pixel analysis, the primary tag (0.6m) at 8m is 15.4px / 1.54 px/cell — marginal at best, and in practice not detected reliably.

The SEARCH state only descends when `tag_status.detected == True` (control loop line 504). Since the tag is never detected at 8m, the drone sits at approach altitude until the 60s `landing_timeout` fires, transitioning to ABORT_LAND. The ABORT_LAND state descends at `final_descent_rate` (0.15 m/s), and during this descent the drone eventually reaches an altitude where the tag becomes visible — but by then it's in the abort path, not the normal landing path.

This makes it impossible to distinguish during debugging whether the drone is descending because it has detected the tag (intentional) or because it timed out (unintentional).

### Desired Behavior

In SEARCH state, when the tag is NOT detected, the drone should descend in **0.5m increments every 3 seconds** until the tag is detected. When the tag IS detected, follow existing logic (tag-confirm then transition to DESCEND_COARSE).

This provides clear visual and log differentiation:
- **Step-down descent (no tag):** periodic 0.5m drops with log messages — clearly "searching" behavior
- **Tag-guided descent (tag detected):** smooth continuous descent — tag has been acquired

### Fix

Modify SEARCH state control loop in `precision_landing_node.py`:
1. Add `search_step_interval` (3.0s) and `search_step_amount` (0.5m) parameters
2. Track `_search_last_stepdown` time
3. When tag NOT detected: step down `search_step_amount` every `search_step_interval`, log each step
4. When tag detected: existing behavior (continuous descent at `search_descent_rate`)
5. Both paths clamp to `min_search_altitude`

---

## ISS-010: tag_detector_adapter_node crashes on tag detection — missing pose field

**Status:** Fixed
**Date:** 2026-03-28
**Severity:** Major
**Component:** tag_detector_adapter_node
**Resolved:** 2026-03-28

### Problem

After ISS-009 fix, the step-down logic continues firing even after the AprilTag should be detected. The drone descends all the way from 4.0m to `min_search_altitude` (1.0m) without ever transitioning to DESCEND_COARSE, despite the tag being within detectable range at lower altitudes.

Log output shows continuous "no tag" step-downs with no tag-confirmed transition:

```
[precision_landing_node-4] [INFO] [1774716948.415601894] [precision_landing]: SEARCH step-down (no tag): target_alt=4.0m
[precision_landing_node-4] [INFO] [1774716951.415811188] [precision_landing]: SEARCH step-down (no tag): target_alt=3.5m
[precision_landing_node-4] [INFO] [1774716954.415899151] [precision_landing]: SEARCH step-down (no tag): target_alt=3.0m
[precision_landing_node-4] [INFO] [1774716957.468379298] [precision_landing]: SEARCH step-down (no tag): target_alt=2.5m
[precision_landing_node-4] [INFO] [1774716960.465846581] [precision_landing]: SEARCH step-down (no tag): target_alt=2.0m
[precision_landing_node-4] [INFO] [1774716963.515349774] [precision_landing]: SEARCH step-down (no tag): target_alt=1.5m
[precision_landing_node-4] [INFO] [1774716966.565936059] [precision_landing]: SEARCH step-down (no tag): target_alt=1.0m
[precision_landing_node-4] [INFO] [1774716969.615673629] [precision_landing]: SEARCH step-down (no tag): target_alt=1.0m
[precision_landing_node-4] [INFO] [1774716972.665878022] [precision_landing]: SEARCH step-down (no tag): target_alt=1.0m
```

### Root Cause — `detection.pose` AttributeError crashes callback

`tag_detector_adapter_node.py` line 190 accesses `detection.pose.pose.pose.position`, but `apriltag_msgs/msg/AprilTagDetection` (v2.0.1) has **no `pose` field**. The message fields are: `family`, `id`, `hamming`, `goodness`, `decision_margin`, `centre`, `corners`, `homography`.

When tags are **not** visible, `_detection_cb` takes the else branch and publishes `detected=False` successfully. When tags **are** visible, `_publish_target()` hits `AttributeError` and crashes before `status_pub.publish(status)` executes — so `detected=True` is **never** published to `/dbvf/tag_status`.

apriltag_ros v3.3.0 publishes pose via TF transforms (`AprilTagNode.cpp:237-251`), not in the detection message. The `_publish_target` method needs to compute pose from the 2D detection data (centre, corners, homography) or subscribe to the TF transforms instead.

### Ruled Out

| Hypothesis | Finding |
|------------|---------|
| **Topic/subscriber mismatch** | Both nodes use `/dbvf/tag_status`, `TagStatus` msg, QoS depth 10 — matched |
| **Debounce too aggressive** | buffer_size=30, threshold=0.8 is aggressive but first detection is immediate (line 36) — not the cause |
| **tag_confirm_frames too high** | Set to 5 — reachable if `detected=True` were ever published |
| **Camera topic wrong** | Launch uses `iris_hardmount_bridge.yaml` with correct `camera_link` path |
| **apriltag_ros config** | family=36h11, IDs=[0,1], sizes=[0.6,0.15] all correct |

### Resolution

Replaced the broken `detection.pose.pose.pose.position` access in `_publish_target()` with homography-based pose estimation computed locally.

1. **Added `estimate_tag_position()` pure function** — Decomposes `K⁻¹ * H` to extract the tag's 3D position in camera frame, matching the apriltag C library's `estimate_pose_for_tag_homography` algorithm. Uses only fields available in `AprilTagDetection.msg` v2.0.1: `homography` (9-element flat array) + camera intrinsics from `CameraInfo` + known `tag_size`.

2. **Fixed `_publish_target()`** — Calls `estimate_tag_position(detection.homography, ...)` instead of accessing the non-existent `detection.pose` field. The resulting camera-frame position is then transformed to body frame via the existing `camera_to_body()`.

3. **Added 6 unit tests** in `test/test_tag_pose_estimation.py` — centered tag, off-center tag, close range, secondary tag size, and homography scale invariance.

All 72 tests pass (66 existing + 6 new). Integration retest needed to confirm `detected=True` is published when tags are in camera view.

---

## ISS-011: Drone touches down before secondary tag can be acquired — needs slow descent near ground

**Status:** Fixed
**Date:** 2026-03-28
**Severity:** Major
**Component:** precision_landing_node (all descent states)
**Resolved:** 2026-03-28

### Problem

The drone does not pause and hover when it detects the secondary AprilTag (ID 1, 0.15m). The 2-second recognition window is insufficient — the drone descends too fast and touches down (or loses sight of the tag) before the code can react to the secondary tag detection.

This is related to but distinct from ISS-007 (FINE_POSITION state). ISS-007 describes adding a dedicated hold-and-align phase. This issue is about a more fundamental need: **regardless of FSM state, the drone must descend much slower when close to the ground** to give all logic (tag switching, fine positioning, confirmation frames) enough time to execute.

### Desired Behavior

When the rangefinder reads ≤ 2m AGL, enforce a significantly reduced descent rate across **all states** (SEARCH, DESCEND, ABORT_LAND, etc.). This is a global safety/timing constraint, not state-specific logic. The rangefinder altitude threshold and slow descent rate should be configurable parameters.

This gives:
- More time for the secondary tag (0.15m) to be detected and confirmed at close range
- More time for tag switching (primary → secondary) debounce to complete
- More time for any future FINE_POSITION logic (ISS-007) to engage before touchdown
- Safer landings in general

### Approach

1. **Read rangefinder altitude** from `/dbvf/vehicle_state` (already published by mavlink_interface_node)
2. **Clamp descent rate** when rangefinder altitude ≤ threshold (default 2.0m) — apply as a global constraint in the control loop, not per-state
3. **Configurable parameters:** `slow_descent_altitude` (default 2.0m), `slow_descent_rate` (TBD — slow enough to give ≥ 4-5s of sub-2m flight time)

### Dependencies

- Rangefinder must be working (ISS-008 fix — awaiting integration retest)
- ISS-010 must be resolved (now fixed) so tag detections actually propagate

### Resolution

Added global rangefinder-based descent rate clamping in `precision_landing_node.py`:

1. **Pure function `clamp_descent_rate()`** — Clamps positive (downward) vz to `slow_descent_rate` when `range_alt ≤ slow_descent_altitude`. Passes through unchanged when above threshold, ascending, holding, or rangefinder invalid (-1.0 sentinel).

2. **Applied in `_call_guided_velocity()`** — The single choke point for all velocity commands. Covers all descent states (DESCEND_COARSE, DESCEND_HOLD, DESCEND_OFFSET, DESCEND_FINAL, SMALL_TAG_SEARCH, ABORT_LAND) globally.

3. **Log-once on entry** — Logs when the clamp first activates (not every tick). Resets when clamp deactivates.

4. **Configurable parameters** added to `sim_params.yaml`:
   - `slow_descent_altitude`: 2.0m (rangefinder threshold)
   - `slow_descent_rate`: 0.1 m/s (gives ~20s from 2m — well above the ≥4s requirement)

5. **8 unit tests** in `test_descent_rate_clamp.py` — covers above threshold, at threshold, below threshold, already slow, zero vz, ascending, invalid rangefinder.

All 81 tests pass (8 new + 73 existing). VehicleState.msg already had `range_alt` field — no message changes needed.

---

## ISS-012: Drone does not switch landing target from large tag to small tag after detection

**Status:** Resolved (2026-03-28)
**Date:** 2026-03-28
**Severity:** Major
**Component:** tag_detector_adapter_node (`select_best_tag`) / precision_landing_node (DESCEND_COARSE → DESCEND_HOLD transition / lateral servo)

### Problem

Even though the secondary AprilTag (ID 1, 0.15m) is detected and confirmed for more than the required 2 seconds (`small_tag_confirm_time`), the drone continues to land centered on the primary tag (ID 0, 0.6m) rather than switching its lateral positioning target to the secondary tag.

The FSM transitions (DESCEND_COARSE → DESCEND_HOLD → DESCEND_OFFSET) appear to fire correctly based on small tag detection, but the actual lateral servo (`_velocity_servo`) continues using the primary tag's position for PID corrections. The drone lands on the center of the large tag instead of the center of the small tag.

### Expected Behavior

Once the secondary tag is confirmed and the FSM enters DESCEND_HOLD/DESCEND_OFFSET, the lateral PID should use the secondary tag's position (from `latest_target`) to center the drone over the small tag. The `offset_forward` / `offset_right` parameters should then apply relative to the small tag center for final payload mechanism alignment.

### Root Cause

`select_best_tag()` in `tag_detector_adapter_node.py` unconditionally prefers the primary tag when both tags are visible. Both tags are within the camera's 114.6° HFOV at all descent altitudes (they're 0.5m apart; ground coverage at 1m is ~3.1m). This causes the debounce to revert to primary within ~0.8s whenever the drone stabilizes, because primary detection recovers and `select_best_tag` feeds primary as the candidate on every frame.

The adapter has no knowledge of the FSM state, so it independently reverts to primary even though the FSM has confirmed secondary and is trying to center on it.

### Fix Attempt 1: Preferred Tag Coordination (2026-03-28)

**Spec:** `docs/superpowers/specs/2026-03-28-preferred-tag-coordination-design.md`

Implemented a `/dbvf/cmd/preferred_tag_id` topic (Int32, 20Hz) so the precision_landing_node can tell the adapter which tag to prioritize:

1. **`select_best_tag()` gained `preferred_id` param** — preferred (if detected) > primary > secondary > None fallback
2. **Adapter subscribes to `/dbvf/cmd/preferred_tag_id`**, passes stored value to `select_best_tag` in `_detection_cb`
3. **Precision landing publishes preferred tag** every control loop tick: `1` (secondary) for DESCEND_HOLD/DESCEND_OFFSET/DESCEND_FINAL/SMALL_TAG_SEARCH, `0` (primary) for all other active states
4. **Safety check in `_velocity_servo`** — rejects `latest_target` where `tag_id != secondary_tag_id` when in DESCEND_HOLD/OFFSET/FINAL
5. **6 new tests** (4 tag selection + 2 velocity servo guard), all 87 tests pass

### Integration Test Result — FAILED

```
DESCEND_COARSE ran ~32s → tag_lost → SEARCH → ABORT_LAND (timeout)
Never reached DESCEND_HOLD.
```

Full log:
```
[precision_landing_node] Starting precision landing at -35.3632531, 149.1657896
[precision_landing_node] APPROACH -> SEARCH (approach_complete)
[precision_landing_node] SEARCH -> DESCEND_COARSE (tag_confirmed)
[precision_landing_node] Slow descent clamp active: range_alt=1.9m, vz 0.30 -> 0.10 m/s
[precision_landing_node] DESCEND_COARSE -> SEARCH (tag_lost)          ← after ~32s, never DESCEND_HOLD
[precision_landing_node] SEARCH step-down (no tag): target_alt=1.9m
[precision_landing_node] SEARCH step-down (no tag): target_alt=1.4m
[precision_landing_node] SEARCH step-down (no tag): target_alt=1.0m
[precision_landing_node] SEARCH step-down (no tag): target_alt=1.0m
[precision_landing_node] SEARCH -> ABORT_LAND (timeout)
```

### Why Fix 1 Failed — Chicken-and-Egg Problem

The preferred tag coordination only activates in DESCEND_HOLD/OFFSET/FINAL/SMALL_TAG_SEARCH. During DESCEND_COARSE, `preferred_id=0` (primary), so the adapter still prefers primary when both tags are visible. The debounce never switches `active_tag_id` to secondary, so `small_tag_detected` stays False, and **DESCEND_HOLD is never reached**.

The design has a circular dependency:
- **DESCEND_HOLD requires** secondary tag to become active in the debounce (for `small_tag_confirm_time`)
- **Secondary becomes active** only when preferred=1 (or primary is not detected)
- **Preferred=1** is only published in DESCEND_HOLD/OFFSET/FINAL/SMALL_TAG_SEARCH

Additionally, the tag was entirely lost after ~32s in DESCEND_COARSE (triggering `tag_lost` → SEARCH). At the slow descent rate (0.1 m/s from 1.9m), the drone may have descended below detectable range or drifted off the tags. This secondary failure needs separate investigation.

### Full Data Path Trace (2026-03-28)

Traced every step of the data flow during DESCEND_COARSE to confirm the circular dependency:

1. `precision_landing_node.py:576-579` — publishes `preferred_id=0` (DESCEND_COARSE not in the secondary-preference set)
2. `tag_detector_adapter_node.py:179-181` — adapter stores `preferred_tag_id=0`
3. `tag_detector_adapter_node.py:57-60` — `select_best_tag(detections, 0, 1, preferred_id=0)`: preferred=0 is in detections (both tags visible at 1.9m), returns `(0, primary_det)` — candidate is always 0
4. `tag_detector_adapter_node.py:39` — debounce: `candidate_id(0) == active_id(0)` → returns 0 immediately, no switch attempt ever occurs
5. `tag_detector_adapter_node.py:206` — `TagStatus.active_tag_id = 0`
6. `precision_landing_node.py:184-185` — `small_tag_detected = (active_tag_id == 1)` → always False
7. `precision_landing_node.py:189` — `small_tag_first_seen` is never set → DESCEND_HOLD transition never fires

The design spec table (preferred-tag-coordination-design.md, line 34-38) maps DESCEND_COARSE → preferred=0, which is the root cause. The system can never escape DESCEND_COARSE because the secondary tag preference is gated behind states that require secondary to already be active.

### Timing Constraints (10fps camera)

Camera is 10fps (iris_with_hardmount_camera model.sdf, update_rate=10). Image is 640x480, HFOV=2.0 rad (114.6°).

| Metric | Value |
|--------|-------|
| Debounce switch time | 24 frames / 10fps = **2.4s** |
| FSM confirm time (`small_tag_confirm_time`) | **2.0s** |
| Total transition (debounce + confirm) | **4.4s** |
| Descent during transition (0.1 m/s) | **0.44m** |
| Available margin (2.0m slow_descent → 1.5m floor) | **0.50m** |
| Net margin with debounce | **0.06m** — too thin |
| Net margin without debounce | **0.30m** — comfortable |

Secondary tag (0.15m) pixel size at altitude:
- 2.5m: 12px (unreliable), 2.0m: 15px (borderline), 1.5m: 21px (reliable)

### Tag Lost After ~32s — Not a Code Bug

The ~32s tag loss was caused by the drone sitting idle while logs were inspected (operator paused the test). Not a software issue.

### Next Steps — Recommended Fix: Hybrid Option 1 + 2

**Change 1 (precision_landing_node.py:576-579):** Publish `preferred_id=1` during DESCEND_COARSE when `vs.range_alt <= slow_descent_altitude` (2.0m). This triggers the adapter to start preferring secondary only when close enough for reliable detection (~15px at 2.0m).

**Change 2 (tag_detector_adapter_node.py, DebounceFilter.update):** Accept a `preferred_id` parameter. When `candidate_id == preferred_id`, immediately set `active_id = candidate_id` (bypass the 80% buffer threshold). Only applies to the forward direction (switching TO preferred). Switching AWAY from preferred still uses full debounce (2.4s), preventing unwanted revert.

**Why both changes are needed:**
- Option 1 alone: 2.4s debounce + 2.0s confirm = 4.4s → 0.06m margin above floor — one frame of jitter triggers SMALL_TAG_SEARCH instead of DESCEND_HOLD.
- Option 1 + 2: 0s debounce + 2.0s confirm = 2.0s → 0.30m margin — robust.

**Why not the other options:**
- Option 2 alone (bypass debounce for all of DESCEND_COARSE): Would need preferred=1 from DESCEND_COARSE entry, but secondary tag may be <10px at higher altitudes (e.g. 5m+). The range_alt threshold ensures reliable detection.
- Option 3 (raw detections in FSM): Requires new field in `dbvf_msgs/TagStatus`, message rebuild, parallel data path. More invasive for no practical benefit.
- Option 4 (lower threshold): Doesn't help — with preferred=0, the candidate is always primary regardless of threshold.

**Flicker protection with debounce bypass:** The 2.0s `small_tag_confirm_time` handles flicker. If secondary detection flickers, `small_tag_first_seen` resets (line 198). The bypass is asymmetric — fast TO preferred, slow AWAY from preferred — so momentary secondary loss causes brief `detected=False` (PID holds position) rather than reverting to primary.

### Fix 2: Hybrid Option 1 + 2 — RESOLVED (2026-03-28)

Implemented the recommended hybrid fix:

**Change 1 — `precision_landing_node.py`:** Extracted `compute_preferred_tag_id()` pure function. Publishes `preferred_id=1` during `DESCEND_COARSE` when `vs.range_alt >= 0.0` AND `vs.range_alt <= slow_descent_altitude` (2.0m). This triggers secondary preference only when the drone is close enough for reliable small tag detection (~15px at 2.0m altitude).

**Change 2 — `tag_detector_adapter_node.py`:** Added `preferred_id=None` parameter to `DebounceFilter.update()`. When `candidate_id is not None` and `candidate_id == preferred_id`, immediately sets `active_id = candidate_id` (bypasses 80% buffer threshold). Only affects forward direction (switching TO preferred); switching away still requires normal debounce. `_detection_cb` passes `self.preferred_tag_id` through.

**Tests added:** 6 new tests (3 debounce filter + 3 preferred tag publish), 93 total tests pass.

**Integration test result — SUCCESS:** Drone descends on primary tag, transitions to secondary tag tracking at ~2.0m altitude, and lands centered on the small tag.
