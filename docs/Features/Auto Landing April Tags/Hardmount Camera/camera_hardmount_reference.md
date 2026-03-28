# Camera Gimbal-to-Hardmount: Connection Points & Interactions

## Goal
Replace the gimbal-mounted camera with a body-fixed (hardmount) camera so that camera orientation tracks the drone fuselage directly.

---

## 1. Gazebo Model Files (SDF)

### 1a. Primary Drone Model — `iris_with_gimbal`
**File:** `src/ardupilot_gazebo/models/iris_with_gimbal/model.sdf`

**What it does:** Composes the drone by merging `iris_with_standoffs` (airframe) + `gimbal_small_3d` (3-DOF gimbal with camera).

**Relevant sections:**
- **Lines ~9-13** — `<include>` of `gimbal_small_3d` sub-model with `merge="true"`
  ```xml
  <include merge="true">
    <uri>package://ardupilot_gazebo/models/gimbal_small_3d</uri>
    <name>gimbal</name>
    <pose degrees="true">0 -0.01 -0.124923 90 0 90</pose>
  </include>
  ```
- **Lines ~15-25** — `gimbal_joint` (revolute with zero limits = fixed) connecting `base_link` → `gimbal_link`
- **Lines ~307-340** — ArduPilotPlugin control channels 8/9/10 mapping to gimbal joints:
  - Channel 8 → `roll_joint` via `/gimbal/cmd_roll`
  - Channel 9 → `pitch_joint` via `/gimbal/cmd_pitch`
  - Channel 10 → `yaw_joint` via `/gimbal/cmd_yaw`
- **JointPositionController plugins** (×3) — drive each gimbal joint with `p_gain=2`

**Action needed:** Replace the gimbal include + gimbal_joint + all gimbal control channels/plugins with a camera sensor link fixed directly to `base_link`.

---

### 1b. Gimbal Sub-Model — `gimbal_small_3d`
**File:** `src/ardupilot_gazebo/models/gimbal_small_3d/model.sdf`

**What it does:** Defines a 3-DOF gimbal (yaw→roll→pitch) with the camera sensor on `pitch_link`.

**Joint chain:**
```
gimbal_link → yaw_joint → yaw_link → roll_joint → roll_link → pitch_joint → pitch_link (camera)
```

**Camera sensor spec (on `pitch_link`):**
- Resolution: 640×480
- HFOV: 2.0 rad (~114°)
- Update rate: 10 Hz
- Clip: 0.05–15000 m
- Pose: `0 0 0 -1.57 -1.57 0` (pointing downward)
- Frame ID: `pitch_link`
- Plugins: `CameraZoomPlugin` (max zoom 125×), `GstCameraPlugin` (UDP 127.0.0.1:5600)

**Action needed:** Extract the camera sensor definition and attach it directly to `base_link` on the drone. The gimbal model itself becomes unnecessary.

---

### 1c. Other Gimbal Models (for reference)
| Model | File | DOF |
|-------|------|-----|
| `gimbal_small_2d` | `src/ardupilot_gazebo/models/gimbal_small_2d/model.sdf` | Roll + Tilt |
| `gimbal_small_1d` | `src/ardupilot_gazebo/models/gimbal_small_1d/model.sdf` | Tilt only |

These are not currently used by the drone model but exist as alternatives.

---

## 2. Gazebo World Files

### 2a. `iris_runway.sdf`
**File:** `src/ardupilot_gazebo/worlds/iris_runway.sdf`
- References `model://iris_with_gimbal` for the drone spawn
- **Action needed:** Update to reference new model (or no change if the model is modified in-place)

### 2b. `iris_warehouse.sdf`
**File:** `src/ardupilot_gazebo/worlds/iris_warehouse.sdf`
- Also references `model://iris_with_gimbal`
- Same consideration as above

### 2c. `gimbal.sdf` (test world)
**File:** `src/ardupilot_gazebo/worlds/gimbal.sdf`
- Standalone gimbal test world — not relevant to drone operation but references gimbal_small_3d

---

## 3. ROS2-Gazebo Bridge Configuration

**File:** `src/ardupilot_gz/ardupilot_gz_bringup/config/iris_bridge.yaml`

**Camera topic mappings (lines ~28-37):**
```yaml
- ros_topic_name: "camera/image"
  gz_topic_name: "/world/map/model/iris/link/pitch_link/sensor/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "camera/camera_info"
  gz_topic_name: "/world/map/model/iris/link/pitch_link/sensor/camera/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS
```

**Action needed:** Change `gz_topic_name` paths from `.../link/pitch_link/sensor/camera/...` to `.../link/camera_link/sensor/camera/...` (or whatever the new camera link is named, e.g. `base_link` if mounted directly there).

---

## 4. Launch Files

### 4a. Single Drone Launch
**File:** `src/ardupilot_gz/ardupilot_gz_bringup/launch/robots/iris.launch.py`
- Spawns `iris_with_gimbal` model (references `package://ardupilot_gazebo/models/iris_with_gimbal/model.sdf`)
- Loads `gazebo-iris-gimbal.parm` for ArduPilot parameters
- Starts `ros_gz_bridge` with `iris_bridge.yaml`
- **Action needed:** Update model reference if creating a new model. Update parameter file reference.

### 4b. Multi-Drone Launch
**File:** `src/ardupilot_gz/ardupilot_gz_bringup/launch/iris_multi_uav.launch.py` (referenced in CLAUDE.md)
- Includes `iris.launch.py` multiple times
- Inherits the model and bridge config from there

### 4c. Runway Launch
**File:** `src/ardupilot_gz/ardupilot_gz_bringup/launch/iris_runway.launch.py`
- Wrapper that includes `robots/iris.launch.py` + world file

---

## 5. ArduPilot Parameters

**File:** `src/ardupilot_gazebo/config/gazebo-iris-gimbal.parm`

**Gimbal-specific parameters:**
```
MNT1_TYPE        1       # Servo gimbal enabled
MNT1_PITCH_MAX   45
MNT1_PITCH_MIN   -135
MNT1_ROLL_MAX    30
MNT1_ROLL_MIN    -30
MNT1_YAW_MAX     160
MNT1_YAW_MIN     -160

RC6_OPTION       212     # Mount1 Roll
RC7_OPTION       213     # Mount1 Pitch
RC8_OPTION       214     # Mount1 Yaw

SERVO9_FUNCTION  8       # Mount1 Roll
SERVO10_FUNCTION 7       # Mount1 Pitch
SERVO11_FUNCTION 6       # Mount1 Yaw
```

**Action needed:** Remove or zero out all `MNT1_*` parameters (set `MNT1_TYPE 0` to disable). Remove the RC and SERVO mappings for gimbal channels. Consider creating a new param file (e.g., `gazebo-iris-hardmount.parm`).

---

## 6. ROS2 Custom Nodes (Formation Control)

### 6a. Circle Detector Node
**File:** `src/formation_control/circle_detector/circle_detector.py`

**Camera subscriptions (lines ~73-82):**
- Subscribes to `/camera/image` (sensor_msgs/Image)
- Subscribes to `/camera/camera_info` (sensor_msgs/CameraInfo)

**Image processing (lines ~291-409):**
- Uses CvBridge for ROS↔OpenCV conversion
- Extracts camera intrinsics (fx, fy, cx, cy) from CameraInfo
- Performs Hough Circle Transform for target detection
- Calculates 3D position from circle radius using focal length

**Action needed:** The ROS topic names (`/camera/image`, `/camera/camera_info`) remain the same — no code changes needed here as long as the bridge config publishes to the same ROS topic names. However, the camera intrinsics may change if the sensor spec changes, and the image behavior will be different (drone tilt = camera tilt) which may affect detection algorithms.

---

## 7. RViz Configuration

**File:** `src/ardupilot_gz/ardupilot_gz_bringup/rviz/iris.rviz`

- Displays `/camera/image` topic
- **Action needed:** No change needed (topic name unchanged)

---

## 8. Summary: What Must Change

| # | Component | File | Change Required |
|---|-----------|------|----------------|
| 1 | **Drone model SDF** | `models/iris_with_gimbal/model.sdf` | Remove gimbal include; add camera link+sensor directly to base_link; remove gimbal control channels from ArduPilotPlugin; remove JointPositionController plugins |
| 2 | **Bridge config** | `config/iris_bridge.yaml` | Update Gazebo topic paths from `pitch_link` to new camera link name |
| 3 | **ArduPilot params** | `config/gazebo-iris-gimbal.parm` | Disable mount (`MNT1_TYPE 0`), remove gimbal servo/RC mappings |
| 4 | **Launch file** | `launch/robots/iris.launch.py` | Update model SDF path and/or param file reference if new files created |
| 5 | **World files** | `worlds/iris_runway.sdf`, `worlds/iris_warehouse.sdf` | Update model URI if creating a new model instead of modifying in-place |

### What Does NOT Need to Change
- Circle detector node (`circle_detector.py`) — same ROS topics
- RViz config — same ROS topics
- Formation control nodes — no camera dependencies
- Formation messages — no camera fields

---

## 9. Camera Sensor Block (for reuse)

This is the camera sensor definition to extract from the gimbal model and attach to the drone body:

```xml
<link name="camera_link">
  <pose degrees="true">0 -0.01 -0.125 0 0 0</pose>  <!-- below fuselage, adjust as needed -->
  <inertial>
    <mass>0.05</mass>
  </inertial>
  <sensor name="camera" type="camera">
    <gz_frame_id>camera_link</gz_frame_id>
    <pose>0 0 0 0 1.5708 0</pose>  <!-- pointing down: positive pitch in Gazebo = downward -->
    <camera>
      <horizontal_fov>2.0</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.05</near>
        <far>15000</far>
      </clip>
    </camera>
    <always_on>1</always_on>
    <update_rate>10</update_rate>
    <visualize>1</visualize>
    <!-- Optional: keep GStreamer streaming -->
    <plugin name="GstCameraPlugin" filename="GstCameraPlugin">
      <udp_host>127.0.0.1</udp_host>
      <udp_port>5600</udp_port>
      <use_basic_pipeline>true</use_basic_pipeline>
      <use_cuda>false</use_cuda>
    </plugin>
  </sensor>
</link>

<!-- Fixed joint to drone body -->
<joint name="camera_joint" type="fixed">
  <parent>base_link</parent>
  <child>camera_link</child>
</joint>
```
