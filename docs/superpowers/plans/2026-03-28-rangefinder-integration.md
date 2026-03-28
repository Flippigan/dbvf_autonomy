# Rangefinder Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a simulated downward ToF rangefinder (matching ARK Flow hardware specs) that feeds altitude data to both ArduPilot SITL and ROS2 VehicleState.

**Architecture:** A `gpu_lidar` sensor on `iris_with_standoffs` publishes LaserScan on `/rangefinder`. Two parallel data paths: (1) ArduPilotPlugin reads it as `rng_1` for SITL → MAVLink RANGEFINDER → mavlink_interface_node → VehicleState.range_alt, and (2) ros_gz_bridge forwards it directly as a ROS2 LaserScan topic for debugging.

**Tech Stack:** Gazebo Harmonic SDF, ArduPilotPlugin C++ (config only), ArduPilot SITL params, ros_gz_bridge YAML, ROS2 Humble messages, Python (pymavlink)

**Design Spec:** `docs/superpowers/specs/2026-03-28-rangefinder-integration-design.md`

---

### Task 1: Add rangefinder sensor to iris_with_standoffs SDF

**Files:**
- Modify: `src/ardupilot_gazebo/models/iris_with_standoffs/model.sdf`

This adds the physical sensor to the base drone model. It creates a lightweight `rangefinder_link` with a single-beam `gpu_lidar` sensor pointing straight down, attached to `base_link` via a fixed joint.

- [ ] **Step 1: Add rangefinder_link and sensor after the navsat_sensor closing tag**

Open `src/ardupilot_gazebo/models/iris_with_standoffs/model.sdf`. After line 165 (`</link>` closing `base_link`), and before `<link name='imu_link'>` (line 167), insert the new rangefinder link and joint:

```xml
    <link name='rangefinder_link'>
      <pose>0 0 -0.05 0 0 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.000001</ixx>
          <iyy>0.000001</iyy>
          <izz>0.000001</izz>
        </inertia>
      </inertial>
      <sensor name='rangefinder' type='gpu_lidar'>
        <pose>0 0 0 0 1.5708 0</pose>
        <topic>rangefinder</topic>
        <update_rate>50</update_rate>
        <lidar>
          <scan>
            <horizontal>
              <samples>1</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </horizontal>
            <vertical>
              <samples>1</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.1</min>
            <max>30.0</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </lidar>
        <visualize>true</visualize>
      </sensor>
    </link>
    <joint name='rangefinder_joint' type='revolute'>
      <child>rangefinder_link</child>
      <parent>base_link</parent>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>0</lower>
          <upper>0</upper>
        </limit>
        <dynamics>
          <damping>1.0</damping>
        </dynamics>
      </axis>
    </joint>
```

Key details:
- `<pose>0 0 -0.05 0 0 0</pose>` on the link — positions sensor 5cm below drone center (underside of body, above landing legs)
- `<pose>0 0 0 0 1.5708 0</pose>` on the sensor — rotates lidar beam from default +X to -Z (straight down). Positive pitch = downward in Gazebo SDF convention.
- `<topic>rangefinder</topic>` — overrides auto-generated topic, publishes to `/rangefinder` in Gazebo
- Single beam: `<samples>1</samples>` horizontal and vertical with `min_angle`/`max_angle` = 0
- Range 0.1–30m matches ARK Flow Broadcom AFBR-S50LV85D specs
- 50 Hz update rate per design spec
- Gaussian noise stddev 0.01m for realistic sensor behavior
- `<visualize>true</visualize>` shows ray in Gazebo for debugging

- [ ] **Step 2: Commit**

```bash
git add src/ardupilot_gazebo/models/iris_with_standoffs/model.sdf
git commit -m "feat: add rangefinder gpu_lidar sensor to iris_with_standoffs model"
```

---

### Task 2: Add sensor block to ArduPilotPlugin config

**Files:**
- Modify: `src/ardupilot_gazebo/models/iris_with_gimbal/model.sdf`

This tells the ArduPilotPlugin to subscribe to the `/rangefinder` Gazebo topic and forward the range data as `rng_1` in the JSON state packet sent to ArduPilot SITL. The plugin's `LoadRangeSensors()` function reads `<sensor>` elements with `<type>`, `<index>`, and `<topic>`.

- [ ] **Step 1: Add sensor block inside the ArduPilotPlugin element**

Open `src/ardupilot_gazebo/models/iris_with_gimbal/model.sdf`. Find the `<!-- Sensors -->` comment and `<imuName>` line inside the `ArduPilotPlugin` plugin block (around line 225). After `<imuName>imu_link::imu_sensor</imuName>`, add:

```xml

      <sensor>
        <type>lidar</type>
        <index>1</index>
        <topic>/rangefinder</topic>
      </sensor>
```

Key details:
- `<type>lidar</type>` — tells the plugin this is a lidar/range sensor (triggers LaserScan subscription)
- `<index>1</index>` — maps to `rng_1` in the JSON state packet to SITL (1-indexed, plugin subtracts 1 internally)
- `<topic>/rangefinder</topic>` — must match the `<topic>` in the gpu_lidar sensor from Task 1
- The plugin calls `RangeCb()` which extracts the minimum range from the LaserScan and stores it in `ranges[0]`

- [ ] **Step 2: Commit**

```bash
git add src/ardupilot_gazebo/models/iris_with_gimbal/model.sdf
git commit -m "feat: add rangefinder sensor block to ArduPilotPlugin config"
```

---

### Task 3: Add RNGFND1 ArduPilot parameters

**Files:**
- Modify: `src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm`

This configures ArduPilot SITL to recognize and use the rangefinder data arriving via the JSON state packet. Without these parameters, ArduPilot ignores the `rng_1` value.

- [ ] **Step 1: Add RNGFND1 parameters at the end of the file**

Open `src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm`. After the last line (`PLND_LAG        0.05`), add:

```
# Rangefinder (SITL simulated, reads from Gazebo via JSON)
RNGFND1_TYPE     100
RNGFND1_MIN_CM   10
RNGFND1_MAX_CM   3000
RNGFND1_ORIENT   25
```

Key details:
- `RNGFND1_TYPE 100` — SITL rangefinder backend (reads `rng_1` from sim JSON state, not a real serial/I2C driver)
- `RNGFND1_MIN_CM 10` — 0.1m minimum (matches Broadcom AFBR-S50LV85D and Gazebo sensor `<min>0.1</min>`)
- `RNGFND1_MAX_CM 3000` — 30m maximum (matches sensor `<max>30.0</max>`)
- `RNGFND1_ORIENT 25` — Downward orientation (MAV_SENSOR_ROTATION_PITCH_270 = pointing straight down)

- [ ] **Step 2: Commit**

```bash
git add src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm
git commit -m "feat: add RNGFND1 ArduPilot parameters for SITL rangefinder"
```

---

### Task 4: Add rangefinder ROS2-Gazebo bridge entry

**Files:**
- Modify: `src/ardupilot_gz/ardupilot_gz_bringup/config/iris_bridge.yaml`

This bridges the Gazebo `/rangefinder` LaserScan topic to ROS2 so it's available for direct consumption and debugging. Follows the exact pattern used by iris_lidar_bridge.yaml for the lidar sensor.

- [ ] **Step 1: Add rangefinder bridge entry at the end of the file**

Open `src/ardupilot_gz/ardupilot_gz_bringup/config/iris_bridge.yaml`. After the last entry (the battery bridge), add:

```yaml

- ros_topic_name: "rangefinder"
  gz_topic_name: "/rangefinder"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
```

Key details:
- `gz_topic_name: "/rangefinder"` — matches the `<topic>rangefinder</topic>` in the Gazebo sensor (Task 1)
- `ros_type_name: "sensor_msgs/msg/LaserScan"` — standard ROS2 laser scan message, same type as lidar bridge
- `direction: GZ_TO_ROS` — one-way, sensor data only flows from simulation to ROS2
- Publishes at sensor rate (50 Hz). For single-beam rangefinder, consumers read `msg.ranges[0]`

- [ ] **Step 2: Commit**

```bash
git add src/ardupilot_gz/ardupilot_gz_bringup/config/iris_bridge.yaml
git commit -m "feat: add rangefinder entry to ROS2-Gazebo bridge config"
```

---

### Task 5: Add range_alt field to VehicleState message

**Files:**
- Modify: `src/dbvf_msgs/msg/VehicleState.msg`

This adds the `range_alt` field that mavlink_interface_node will populate from the RANGEFINDER MAVLink message. Must be built before writing tests or implementation code.

- [ ] **Step 1: Add range_alt field to VehicleState.msg**

Open `src/dbvf_msgs/msg/VehicleState.msg`. After the last line (`float64 heading`), add:

```
float64 range_alt    # Rangefinder AGL altitude (meters), -1.0 if no valid reading
```

The complete file should now be:
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
float64 range_alt    # Rangefinder AGL altitude (meters), -1.0 if no valid reading
```

- [ ] **Step 2: Rebuild dbvf_msgs to generate the updated Python bindings**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dbvf_msgs
source install/setup.bash
```

Expected: Build succeeds. The new `range_alt` field is now available via `from dbvf_msgs.msg import VehicleState`.

- [ ] **Step 3: Verify the field exists in the generated message**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 interface show dbvf_msgs/msg/VehicleState
```

Expected output includes `float64 range_alt` at the end.

- [ ] **Step 4: Commit**

```bash
git add src/dbvf_msgs/msg/VehicleState.msg
git commit -m "feat: add range_alt field to VehicleState message"
```

---

### Task 6: Write failing rangefinder unit tests (TDD red phase)

**Files:**
- Modify: `src/dbvf_autonomy/test/test_mavlink_messages.py`

Write tests BEFORE the implementation. These test two pure functions/constants that will be exported from mavlink_interface_node: `RANGE_ALT_SENTINEL` (the default value) and `extract_rangefinder_distance()` (a testable extraction function).

- [ ] **Step 1: Add rangefinder test imports and tests to test_mavlink_messages.py**

Open `src/dbvf_autonomy/test/test_mavlink_messages.py`. Update the import at the top to also import the new symbols:

Change:
```python
from dbvf_autonomy.mavlink_interface_node import (
    ARDUPILOT_MODE_MAP,
    build_landing_target_params,
)
```

To:
```python
from dbvf_autonomy.mavlink_interface_node import (
    ARDUPILOT_MODE_MAP,
    RANGE_ALT_SENTINEL,
    build_landing_target_params,
    extract_rangefinder_distance,
)
```

Then add two new test functions at the end of the file:

```python


def test_range_alt_sentinel_before_data():
    """range_alt is -1.0 before first RANGEFINDER message received."""
    assert RANGE_ALT_SENTINEL == -1.0


def test_range_alt_populated_from_rangefinder():
    """range_alt populated correctly from RANGEFINDER MAVLink message."""
    class FakeRangefinderMsg:
        distance = 5.43

    assert extract_rangefinder_distance(FakeRangefinderMsg()) == 5.43
```

- [ ] **Step 2: Run tests to verify they FAIL**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
cd /home/finn/Documents/ardu_ws
python -m pytest src/dbvf_autonomy/test/test_mavlink_messages.py -v
```

Expected: FAIL — `ImportError: cannot import name 'RANGE_ALT_SENTINEL'` and `cannot import name 'extract_rangefinder_distance'` from `dbvf_autonomy.mavlink_interface_node`. This confirms the tests are correctly written and waiting for the implementation.

- [ ] **Step 3: Commit the failing tests**

```bash
git add src/dbvf_autonomy/test/test_mavlink_messages.py
git commit -m "test: add rangefinder unit tests (red phase, imports will fail until implementation)"
```

---

### Task 7: Implement RANGEFINDER parsing in mavlink_interface_node (TDD green phase)

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py`

Add the sentinel constant, extraction function, RANGEFINDER message parsing in the read loop, and VehicleState population. This makes the tests from Task 6 pass.

- [ ] **Step 1: Add RANGE_ALT_SENTINEL constant and extract_rangefinder_distance function**

Open `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py`. After the `_MODE_NUM_TO_NAME` dict (line 25) and before the `build_landing_target_params` function (line 28), add:

```python

RANGE_ALT_SENTINEL = -1.0


def extract_rangefinder_distance(msg):
    """Extract distance in meters from a RANGEFINDER MAVLink message."""
    return msg.distance

```

- [ ] **Step 2: Initialize range_alt in __init__**

In the `MavlinkInterfaceNode.__init__` method, after `self.vehicle_heading = 0.0` (line 66) and before `self.last_heartbeat_time = 0.0` (line 67), add:

```python
        self.range_alt = RANGE_ALT_SENTINEL
```

- [ ] **Step 3: Parse RANGEFINDER message in the _read_timer loop**

In the `_read_timer` method, after the `elif mtype == 'GLOBAL_POSITION_INT':` block (which ends around line 159 with `self.vehicle_heading = msg.hdg / 100.0`), add a new elif branch:

```python
                elif mtype == 'RANGEFINDER':
                    self.range_alt = extract_rangefinder_distance(msg)
```

- [ ] **Step 4: Populate range_alt in the VehicleState message**

In the `_read_timer` method, after `state.heading = self.vehicle_heading` (line 172) and before `self.state_pub.publish(state)` (line 173), add:

```python
        state.range_alt = self.range_alt
```

- [ ] **Step 5: Run tests to verify they PASS**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
cd /home/finn/Documents/ardu_ws
python -m pytest src/dbvf_autonomy/test/test_mavlink_messages.py -v
```

Expected: All 5 tests pass (3 existing + 2 new):
```
test_mode_mapping_common_modes PASSED
test_landing_target_angles_only PASSED
test_landing_target_with_position PASSED
test_range_alt_sentinel_before_data PASSED
test_range_alt_populated_from_rangefinder PASSED
```

- [ ] **Step 6: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py
git commit -m "feat: parse RANGEFINDER MAVLink message and populate VehicleState.range_alt"
```

---

### Task 8: Full build and test suite

**Files:** None (build verification only)

Build all modified packages and run the full test suite to ensure nothing is broken.

- [ ] **Step 1: Build all DBVF packages**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dbvf_msgs dbvf_autonomy
source install/setup.bash
```

Expected: Both packages build successfully with no errors.

- [ ] **Step 2: Build ardupilot_gazebo (for model SDF changes)**

```bash
colcon build --packages-select ardupilot_gazebo
```

Expected: Build succeeds. The updated SDF models are installed to the workspace.

- [ ] **Step 3: Run full test suite**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon test --packages-select dbvf_autonomy
colcon test-result --verbose
```

Expected: All 40 tests pass (38 existing + 2 new rangefinder tests). Zero failures.

- [ ] **Step 4: Commit (if any build fixes were needed)**

Only if changes were required. Otherwise skip.

---

### Task 9: Integration smoke test

**Prerequisites:** Full simulation environment (Gazebo + ArduPilot SITL + MAVProxy). This is a manual verification — run these checks to confirm end-to-end data flow.

- [ ] **Step 1: Launch Gazebo + ArduPilot SITL**

```bash
# Terminal 1
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py rviz:=true use_gz_tf:=true
```

Expected: Gazebo opens with drone on runway. A **green/blue rangefinder ray** should be visible pointing straight down from the drone body. If no ray is visible, check the sensor `<visualize>true</visualize>` setting and that `ardupilot_gazebo` was rebuilt.

- [ ] **Step 2: Launch precision landing stack**

```bash
# Terminal 2
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch dbvf_autonomy precision_landing_sim.launch.py
```

- [ ] **Step 3: Verify ROS2 rangefinder topic**

```bash
# Terminal 3
ros2 topic echo /rangefinder --once
```

Expected: A `sensor_msgs/msg/LaserScan` message with `ranges[0]` showing a reasonable distance (roughly the drone's height above ground, ~0.2m if sitting on the runway).

- [ ] **Step 4: Verify VehicleState.range_alt**

```bash
ros2 topic echo /dbvf/vehicle_state --field range_alt --once
```

Expected: A value close to the drone's AGL altitude. Initially `-1.0` is acceptable if no RANGEFINDER MAVLink message has been received yet (ArduPilot needs a moment to start streaming). After a few seconds, should show a positive altitude value.

- [ ] **Step 5: Arm and fly to verify altitude tracking**

```bash
# Terminal 4 — MAVProxy
mavproxy.py --master udpin:0.0.0.0:14550 --console
```

In MAVProxy:
```
mode guided
arm throttle
takeoff 10
```

Then monitor:
```bash
# Terminal 3
ros2 topic echo /dbvf/vehicle_state --field range_alt
```

Expected: `range_alt` tracks upward during takeoff, stabilizes near 10.0 at hover, tracks downward during landing. Values should closely match `alt_rel` but may differ slightly due to terrain and sensor noise.

- [ ] **Step 6: Verify ArduPilot sees the rangefinder (MAVProxy)**

In MAVProxy console:
```
status rangefinder1
```

Expected: Shows rangefinder distance matching the drone's AGL altitude. If it shows 0 or no output, the ArduPilotPlugin `<sensor>` block or RNGFND1 parameters may be misconfigured.
