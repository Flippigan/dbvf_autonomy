# Camera Hardmount Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the gimbal-mounted camera with a body-fixed (hardmount) camera so camera orientation tracks the drone fuselage directly, enabling reliable downward-facing AprilTag detection for auto-landing.

**Architecture:** Create a new drone model (`iris_with_hardmount_camera`) that reuses the existing `iris_with_standoffs` airframe but replaces the 3-DOF gimbal with a single fixed camera link. New param file disables gimbal servos. New bridge config routes the camera topic through the renamed link. Launch and world files are updated to use the new model.

**Tech Stack:** Gazebo SDF 1.9, ROS2 `ros_gz_bridge`, ArduPilot SITL parameters, Python launch files

---

## File Structure

### New Files (Create)
| File | Responsibility |
|------|---------------|
| `src/ardupilot_gazebo/models/iris_with_hardmount_camera/model.config` | Gazebo model metadata for the hardmount variant |
| `src/ardupilot_gazebo/models/iris_with_hardmount_camera/model.sdf` | Drone model: iris_with_standoffs + fixed camera_link (no gimbal) |
| `src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm` | ArduPilot params: frame/motor config only, no gimbal mount |
| `src/ardupilot_gz/ardupilot_gz_bringup/config/iris_hardmount_bridge.yaml` | Bridge config with `camera_link` Gazebo topic paths |

### Modified Files
| File | Change |
|------|--------|
| `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf:76` | Change model URI from `iris_with_gimbal` to `iris_with_hardmount_camera` |
| `src/ardupilot_gz/ardupilot_gz_bringup/launch/robots/iris.launch.py:89,119,144` | Point to new param file, new model SDF, new bridge config |

### Unchanged Files (verified)
| File | Why unchanged |
|------|--------------|
| `src/formation_control/circle_detector/circle_detector.py` | Subscribes to `/camera/image` and `/camera/camera_info` — ROS topic names stay the same |
| `src/ardupilot_gz/ardupilot_gz_bringup/rviz/iris.rviz` | Displays `/camera/image` — ROS topic name unchanged |
| `src/ardupilot_gazebo/models/iris_with_gimbal/` | Preserved as-is for future gimbal use |
| `src/ardupilot_gazebo/models/gimbal_small_3d/` | Preserved as-is |
| `src/ardupilot_gazebo/config/gazebo-iris-gimbal.parm` | Preserved as-is |
| `src/ardupilot_gz/ardupilot_gz_bringup/config/iris_bridge.yaml` | Preserved as-is for gimbal variant |
| `src/ardupilot_gazebo/worlds/iris_runway.sdf` | Not used by ROS2 launch system (launch loads from `ardupilot_gz_gazebo/worlds/`). Only relevant for standalone Gazebo usage. Still references `iris_with_gimbal` — update separately if needed for non-ROS2 workflows |
| `src/ardupilot_gazebo/worlds/iris_warehouse.sdf` | Same as above — not in the active launch path |

---

## Important Context

### World Name
The launch system loads `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf` (world name: **`map`**), NOT `src/ardupilot_gazebo/worlds/iris_runway.sdf` (world name: `iris_runway`). The bridge config Gazebo topic paths use `/world/map/...`.

### Model Spawn Name
The world file spawns the drone as `<name>iris</name>`. The bridge topics use `/world/map/model/iris/...`. This stays the same.

### Camera Gazebo Topic Path
- **Current (gimbal):** `/world/map/model/iris/link/pitch_link/sensor/camera/image`
- **New (hardmount):** `/world/map/model/iris/link/camera_link/sensor/camera/image`

### Camera Orientation
The camera sensor pose `0 0 0 0 -1.5708 0` pitches the camera -90 degrees (straight down). The camera_link is positioned at `0 -0.01 -0.125` relative to base_link (below fuselage center). Since the camera is now body-fixed, drone tilt = camera tilt — this is intentional for AprilTag landing where the drone is roughly level during approach.

---

## Task 1: Create Hardmount ArduPilot Parameter File

**Files:**
- Create: `src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm`

- [ ] **Step 1: Create the parameter file**

```
# Iris is X frame
FRAME_CLASS      1
FRAME_TYPE       1

# Match servo out for motors
MOT_PWM_MIN      1100
MOT_PWM_MAX      1900

# No gimbal mount
MNT1_TYPE        0
```

This keeps the frame and motor config identical to the gimbal variant. `MNT1_TYPE 0` explicitly disables the gimbal mount. All `RC6_*`/`RC7_*`/`RC8_*` gimbal RC inputs and `SERVO9`/`SERVO10`/`SERVO11` gimbal servo outputs are omitted (they default to disabled).

- [ ] **Step 2: Verify file exists and content is correct**

Run: `cat src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm`

Expected: File contents match above with frame, motor, and `MNT1_TYPE 0` params.

- [ ] **Step 3: Commit**

```bash
git add src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm
git commit -m "feat: add hardmount camera ArduPilot param file

Disables gimbal mount (MNT1_TYPE 0) and removes gimbal servo/RC
mappings. Keeps frame class and motor PWM limits identical to
gazebo-iris-gimbal.parm."
```

---

## Task 2: Create Hardmount Drone Model (model.config)

**Files:**
- Create: `src/ardupilot_gazebo/models/iris_with_hardmount_camera/model.config`

- [ ] **Step 1: Create the model directory**

Run: `mkdir -p src/ardupilot_gazebo/models/iris_with_hardmount_camera`

- [ ] **Step 2: Create model.config**

```xml
<?xml version="1.0"?>
<model>
  <name>Iris with Hardmount Camera</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>

  <description>
    Iris quadcopter with body-fixed downward-facing camera.
    Based on iris_with_standoffs airframe.
    No gimbal — camera orientation tracks drone fuselage.
  </description>
  <depend>
    <model>
      <uri>model://iris_with_standoffs</uri>
      <version>2.0</version>
    </model>
  </depend>
</model>
```

- [ ] **Step 3: Commit**

```bash
git add src/ardupilot_gazebo/models/iris_with_hardmount_camera/model.config
git commit -m "feat: add iris_with_hardmount_camera model metadata"
```

---

## Task 3: Create Hardmount Drone Model (model.sdf)

This is the core task. The SDF is derived from `iris_with_gimbal/model.sdf` (367 lines) with these changes:
- **Removed:** gimbal_small_3d include, gimbal_joint, ArduPilotPlugin channels 8-10, JointPositionController plugins (3)
- **Added:** camera_link with sensor, camera_joint (fixed to base_link)
- **Kept:** Everything else (airframe include, lift-drag, apply-joint-force, battery, ArduPilotPlugin motor channels 0-3)

**Camera sensor pose note:** The original gimbal camera uses sensor pose `0 0 0 -1.57 -1.57 0` (roll AND pitch rotated) because the sensor sits on `pitch_link` at the end of a yaw→roll→pitch kinematic chain, and the gimbal itself is mounted with a `90 0 90` degree rotation relative to `base_link`. For the hardmount, the camera_link is directly attached to `base_link` with no intermediate rotations, so only a single -90° pitch (`0 0 0 0 -1.5708 0`) is needed to point the camera straight down. Do NOT copy the original gimbal pose — it would produce the wrong orientation in the hardmount frame.

**Files:**
- Create: `src/ardupilot_gazebo/models/iris_with_hardmount_camera/model.sdf`

- [ ] **Step 1: Create model.sdf**

```xml
<?xml version='1.0'?>
<sdf version="1.9">
  <model name="iris_with_hardmount_camera">
    <include merge="true">
      <uri>package://ardupilot_gazebo/models/iris_with_standoffs</uri>
      <name>iris</name>
    </include>

    <!-- Body-fixed downward-facing camera -->
    <link name="camera_link">
      <pose>0 -0.01 -0.125 0 0 0</pose>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.00001</iyy><iyz>0</iyz>
          <izz>0.00001</izz>
        </inertia>
      </inertial>
      <sensor name="camera" type="camera">
        <gz_frame_id>camera_link</gz_frame_id>
        <pose>0 0 0 0 -1.5708 0</pose>
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
        <plugin name="GstCameraPlugin" filename="GstCameraPlugin">
          <udp_host>127.0.0.1</udp_host>
          <udp_port>5600</udp_port>
          <use_basic_pipeline>true</use_basic_pipeline>
          <use_cuda>false</use_cuda>
        </plugin>
      </sensor>
    </link>

    <joint name="camera_joint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
    </joint>

    <!-- plugins -->
    <plugin filename="gz-sim-joint-state-publisher-system"
      name="gz::sim::systems::JointStatePublisher">
    </plugin>

    <plugin
      filename="gz-sim-odometry-publisher-system"
      name="gz::sim::systems::OdometryPublisher">
      <odom_frame>odom</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <dimensions>3</dimensions>
    </plugin>

    <plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
      <a0>0.3</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.0</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.002</area>
      <air_density>1.2041</air_density>
      <cp>0.084 0 0</cp>
      <forward>0 1 0</forward>
      <upward>0 0 1</upward>
      <link_name>rotor_0</link_name>
    </plugin>
    <plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
      <a0>0.3</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.0</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.002</area>
      <air_density>1.2041</air_density>
      <cp>-0.084 0 0</cp>
      <forward>0 -1 0</forward>
      <upward>0 0 1</upward>
      <link_name>rotor_0</link_name>
    </plugin>

    <plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
      <a0>0.3</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.0</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.002</area>
      <air_density>1.2041</air_density>
      <cp>0.084 0 0</cp>
      <forward>0 1 0</forward>
      <upward>0 0 1</upward>
      <link_name>rotor_1</link_name>
    </plugin>
    <plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
      <a0>0.3</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.0</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.002</area>
      <air_density>1.2041</air_density>
      <cp>-0.084 0 0</cp>
      <forward>0 -1 0</forward>
      <upward>0 0 1</upward>
      <link_name>rotor_1</link_name>
    </plugin>

    <plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
      <a0>0.3</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.0</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.002</area>
      <air_density>1.2041</air_density>
      <cp>0.084 0 0</cp>
      <forward>0 -1 0</forward>
      <upward>0 0 1</upward>
      <link_name>rotor_2</link_name>
    </plugin>
    <plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
      <a0>0.3</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.0</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.002</area>
      <air_density>1.2041</air_density>
      <cp>-0.084 0 0</cp>
      <forward>0 1 0</forward>
      <upward>0 0 1</upward>
      <link_name>rotor_2</link_name>
    </plugin>

    <plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
      <a0>0.3</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.0</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.002</area>
      <air_density>1.2041</air_density>
      <cp>0.084 0 0</cp>
      <forward>0 -1 0</forward>
      <upward>0 0 1</upward>
      <link_name>rotor_3</link_name>
    </plugin>
    <plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
      <a0>0.3</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.0</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.002</area>
      <air_density>1.2041</air_density>
      <cp>-0.084 0 0</cp>
      <forward>0 1 0</forward>
      <upward>0 0 1</upward>
      <link_name>rotor_3</link_name>
    </plugin>

    <plugin filename="gz-sim-apply-joint-force-system"
      name="gz::sim::systems::ApplyJointForce">
      <joint_name>rotor_0_joint</joint_name>
    </plugin>
    <plugin filename="gz-sim-apply-joint-force-system"
      name="gz::sim::systems::ApplyJointForce">
      <joint_name>rotor_1_joint</joint_name>
    </plugin>
    <plugin filename="gz-sim-apply-joint-force-system"
      name="gz::sim::systems::ApplyJointForce">
      <joint_name>rotor_2_joint</joint_name>
    </plugin>
    <plugin filename="gz-sim-apply-joint-force-system"
      name="gz::sim::systems::ApplyJointForce">
      <joint_name>rotor_3_joint</joint_name>
    </plugin>

    <plugin filename="gz-sim-linearbatteryplugin-system"
      name="gz::sim::systems::LinearBatteryPlugin">
      <battery_name>lipo_3500mAh</battery_name>
      <fix_issue_225>true</fix_issue_225>
      <open_circuit_voltage_constant_coef>12.6</open_circuit_voltage_constant_coef>
      <open_circuit_voltage_linear_coef>-2.7</open_circuit_voltage_linear_coef>
      <capacity>3.5</capacity>
      <voltage>12.6</voltage>
      <initial_charge>3.5</initial_charge>
      <power_load>200.0</power_load>
      <start_draining>false</start_draining>
    </plugin>

    <plugin name="ArduPilotPlugin"
      filename="ArduPilotPlugin">
      <!-- Port settings -->
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9002</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>

      <!-- Frame conventions -->
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>

      <!-- Sensors -->
      <imuName>imu_link::imu_sensor</imuName>

      <control channel="0">
        <jointName>rotor_0_joint</jointName>
        <useForce>1</useForce>
        <multiplier>838</multiplier>
        <offset>0</offset>
        <servo_min>1100</servo_min>
        <servo_max>1900</servo_max>
        <type>VELOCITY</type>
        <p_gain>0.20</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>

      <control channel="1">
        <jointName>rotor_1_joint</jointName>
        <useForce>1</useForce>
        <multiplier>838</multiplier>
        <offset>0</offset>
        <servo_min>1100</servo_min>
        <servo_max>1900</servo_max>
        <type>VELOCITY</type>
        <p_gain>0.20</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>

      <control channel="2">
        <jointName>rotor_2_joint</jointName>
        <useForce>1</useForce>
        <multiplier>-838</multiplier>
        <offset>0</offset>
        <servo_min>1100</servo_min>
        <servo_max>1900</servo_max>
        <type>VELOCITY</type>
        <p_gain>0.20</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>

      <control channel="3">
        <jointName>rotor_3_joint</jointName>
        <useForce>1</useForce>
        <multiplier>-838</multiplier>
        <offset>0</offset>
        <servo_min>1100</servo_min>
        <servo_max>1900</servo_max>
        <type>VELOCITY</type>
        <p_gain>0.20</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>

      <!-- No gimbal control channels (8, 9, 10) — camera is body-fixed -->

    </plugin>

    <!-- No JointPositionController plugins — no gimbal joints to drive -->

  </model>
</sdf>
```

**Key differences from `iris_with_gimbal/model.sdf`:**
- Line 3: Model name changed to `iris_with_hardmount_camera`
- Lines 9-25 replaced: Gimbal include + gimbal_joint → camera_link + camera_joint
- Lines 306-340 removed: ArduPilotPlugin channels 8/9/10 (gimbal roll/pitch/yaw)
- Lines 344-364 removed: Three JointPositionController plugins for gimbal joints

- [ ] **Step 2: Verify SDF is well-formed**

Run: `xmllint --noout src/ardupilot_gazebo/models/iris_with_hardmount_camera/model.sdf && echo "Valid XML"`

Expected: `Valid XML` (no errors)

- [ ] **Step 3: Spot-check critical sections**

Run: `grep -c "gimbal\|pitch_joint\|roll_joint\|yaw_joint\|JointPositionController" src/ardupilot_gazebo/models/iris_with_hardmount_camera/model.sdf`

Expected: `0` — no gimbal references remain

Run: `grep -c "camera_link\|camera_joint" src/ardupilot_gazebo/models/iris_with_hardmount_camera/model.sdf`

Expected: `5` or more — camera link and joint are present

- [ ] **Step 4: Commit**

```bash
git add src/ardupilot_gazebo/models/iris_with_hardmount_camera/model.sdf
git commit -m "feat: add iris_with_hardmount_camera model SDF

Body-fixed downward camera on camera_link, fixed-joint to base_link.
Removes all gimbal joints, gimbal ArduPilotPlugin channels (8-10),
and JointPositionController plugins. Motor channels 0-3, lift-drag,
and battery plugins identical to iris_with_gimbal."
```

---

## Task 4: Create Hardmount Bridge Configuration

The bridge config maps Gazebo topic paths to ROS2 topic names. The only change from `iris_bridge.yaml` is the camera topic paths: `pitch_link` → `camera_link`.

**Files:**
- Create: `src/ardupilot_gz/ardupilot_gz_bringup/config/iris_hardmount_bridge.yaml`

- [ ] **Step 1: Create bridge config**

Copy `iris_bridge.yaml` and change the two camera topic Gazebo paths:

```yaml
---
- ros_topic_name: "clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
- ros_topic_name: "joint_states"
  gz_topic_name: "/world/map/model/iris/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS
- ros_topic_name: "odometry"
  gz_topic_name: "/model/iris/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
- ros_topic_name: "gz/tf"
  gz_topic_name: "/model/iris/pose"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS
- ros_topic_name: "gz/tf_static"
  gz_topic_name: "/model/iris/pose_static"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS

- ros_topic_name: "camera/image"
  gz_topic_name: "/world/map/model/iris/link/camera_link/sensor/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
- ros_topic_name: "camera/camera_info"
  gz_topic_name: "/world/map/model/iris/link/camera_link/sensor/camera/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS

- ros_topic_name: "air_pressure"
  gz_topic_name: "/world/map/model/iris/link/base_link/sensor/air_pressure_sensor/air_pressure"
  ros_type_name: "sensor_msgs/msg/FluidPressure"
  gz_type_name: "gz.msgs.FluidPressure"
  direction: GZ_TO_ROS

- ros_topic_name: "imu"
  gz_topic_name: "/world/map/model/iris/link/imu_link/sensor/imu_sensor/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

- ros_topic_name: "magnetometer"
  gz_topic_name: "/world/map/model/iris/link/base_link/sensor/magnetometer_sensor/magnetometer"
  ros_type_name: "sensor_msgs/msg/MagneticField"
  gz_type_name: "gz.msgs.Magnetometer"
  direction: GZ_TO_ROS

- ros_topic_name: "navsat"
  gz_topic_name: "/world/map/model/iris/link/base_link/sensor/navsat_sensor/navsat"
  ros_type_name: "sensor_msgs/msg/NavSatFix"
  gz_type_name: "gz.msgs.NavSat"
  direction: GZ_TO_ROS

- ros_topic_name: "battery"
  gz_topic_name: "/model/iris/battery/linear_battery/state"
  ros_type_name: "sensor_msgs/msg/BatteryState"
  gz_type_name: "gz.msgs.BatteryState"
  direction: GZ_TO_ROS
```

The **only** difference from `iris_bridge.yaml` is on the two camera lines:
- `link/pitch_link/sensor` → `link/camera_link/sensor`

- [ ] **Step 2: Verify the diff is exactly what we expect**

Run: `diff src/ardupilot_gz/ardupilot_gz_bringup/config/iris_bridge.yaml src/ardupilot_gz/ardupilot_gz_bringup/config/iris_hardmount_bridge.yaml`

Expected: Only two lines differ (the `gz_topic_name` lines for camera/image and camera/camera_info), changing `pitch_link` to `camera_link`. The commented-out air_speed and altimeter sections are cleaned up (optional).

- [ ] **Step 3: Commit**

```bash
git add src/ardupilot_gz/ardupilot_gz_bringup/config/iris_hardmount_bridge.yaml
git commit -m "feat: add bridge config for hardmount camera

Maps camera topics through camera_link instead of pitch_link.
All other sensor bridges unchanged from iris_bridge.yaml."
```

---

## Task 5: Update World File

Change the drone model reference in the world file loaded by the launch system.

**Files:**
- Modify: `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf:76`

- [ ] **Step 1: Verify current state**

Run: `grep "iris_with_gimbal" src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf`

Expected: `<uri>model://iris_with_gimbal</uri>`

- [ ] **Step 2: Change model reference**

In `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf`, change line 76:

```xml
<!-- Before -->
<uri>model://iris_with_gimbal</uri>

<!-- After -->
<uri>model://iris_with_hardmount_camera</uri>
```

- [ ] **Step 3: Verify change**

Run: `grep "iris_with_hardmount_camera" src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf`

Expected: `<uri>model://iris_with_hardmount_camera</uri>`

- [ ] **Step 4: Commit**

```bash
git add src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf
git commit -m "feat: use hardmount camera model in iris_runway world"
```

---

## Task 6: Update Launch File

Point the launch file at the new model, parameter file, and bridge config.

**Files:**
- Modify: `src/ardupilot_gz/ardupilot_gz_bringup/launch/robots/iris.launch.py:89,119,144`

- [ ] **Step 1: Change parameter file reference (line 89)**

```python
# Before (line 89)
"gazebo-iris-gimbal.parm",

# After
"gazebo-iris-hardmount.parm",
```

- [ ] **Step 2: Change model SDF reference (line 119)**

```python
# Before (line 119)
pkg_ardupilot_gazebo, "models", "iris_with_gimbal", "model.sdf"

# After
pkg_ardupilot_gazebo, "models", "iris_with_hardmount_camera", "model.sdf"
```

- [ ] **Step 3: Change bridge config reference (line 144)**

```python
# Before (line 144)
pkg_project_bringup, "config", "iris_bridge.yaml"

# After
pkg_project_bringup, "config", "iris_hardmount_bridge.yaml"
```

- [ ] **Step 4: Verify all three references are updated**

Run: `grep -n "hardmount" src/ardupilot_gz/ardupilot_gz_bringup/launch/robots/iris.launch.py`

Expected: Three matches — param file, model path, bridge config.

Run: `grep -n "gimbal\|iris_bridge.yaml" src/ardupilot_gz/ardupilot_gz_bringup/launch/robots/iris.launch.py`

Expected: No matches — all old references removed.

- [ ] **Step 5: Commit**

```bash
git add src/ardupilot_gz/ardupilot_gz_bringup/launch/robots/iris.launch.py
git commit -m "feat: point iris launch at hardmount model, params, and bridge

Updates three references in iris.launch.py:
- Model: iris_with_gimbal → iris_with_hardmount_camera
- Params: gazebo-iris-gimbal.parm → gazebo-iris-hardmount.parm
- Bridge: iris_bridge.yaml → iris_hardmount_bridge.yaml"
```

---

## Task 7: Build and Verify

- [ ] **Step 1: Build the workspace**

Run:
```bash
cd /home/finn/Documents/ardu_ws
colcon build
source install/setup.bash
```

Expected: Build succeeds with no errors. Warnings about other packages are OK.

- [ ] **Step 2: Verify new files are installed**

Run:
```bash
# Check model is installed
ls install/ardupilot_gazebo/share/ardupilot_gazebo/models/iris_with_hardmount_camera/model.sdf

# Check param file is installed
ls install/ardupilot_gazebo/share/ardupilot_gazebo/config/gazebo-iris-hardmount.parm

# Check bridge config is installed
ls install/ardupilot_gz_bringup/share/ardupilot_gz_bringup/config/iris_hardmount_bridge.yaml
```

Expected: All three files exist in the install directory.

- [ ] **Step 3: Commit (if any build fixes were needed)**

Only if changes were required to fix build issues.

---

## Task 8: Smoke Test — Launch Simulation and Verify Camera

- [ ] **Step 1: Launch single-drone simulation**

Run:
```bash
source /home/finn/Documents/ardu_ws/install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py rviz:=true use_gz_tf:=true
```

Expected: Gazebo opens with drone on runway. No errors about missing models or plugins. The drone should have no visible gimbal mechanism — just the airframe with a small camera underneath.

- [ ] **Step 2: Check camera topics are published**

In a new terminal:
```bash
source /home/finn/Documents/ardu_ws/install/setup.bash
ros2 topic list | grep camera
```

Expected:
```
/camera/image
/camera/camera_info
```

- [ ] **Step 3: Verify camera image is being received**

Run:
```bash
ros2 topic hz /camera/image
```

Expected: ~10 Hz update rate (matching the `<update_rate>10</update_rate>` in the SDF).

- [ ] **Step 4: Verify camera_info is being received**

Run:
```bash
ros2 topic echo /camera/camera_info --once
```

Expected: CameraInfo message with width=640, height=480, and populated intrinsic matrix.

- [ ] **Step 5: Visual check in RViz**

In RViz, add an Image display subscribing to `/camera/image`. Verify:
- Image shows the runway/ground below the drone
- Image tilts when the drone tilts (confirming body-fixed mount)
- No black/empty frames

- [ ] **Step 6: Verify no gimbal topics remain**

Run:
```bash
gz topic -l | grep gimbal
ros2 topic list | grep gimbal
```

Expected: No gimbal-related topics.

---

## Rollback Procedure

To revert to gimbal mode, undo the three launch file references and one world file reference:

1. `iris.launch.py:89` → `"gazebo-iris-gimbal.parm"`
2. `iris.launch.py:119` → `"iris_with_gimbal", "model.sdf"`
3. `iris.launch.py:144` → `"iris_bridge.yaml"`
4. `iris_runway.sdf:76` → `model://iris_with_gimbal`

Or simply: `git revert <commit-hash>` for the launch and world file commits.

The new model, param file, and bridge config can remain in the repo — they don't affect anything unless referenced.

---

## Notes for Multi-Drone Extension

When extending this to `iris_multi_uav.launch.py` (5 drones):

1. Each drone's bridge config already uses namespaced model names (`iris_9002`, `iris_9012`, etc.)
2. The Gazebo topic paths will be `/world/map/model/iris_900N/link/camera_link/sensor/camera/image`
3. You'll need per-drone bridge configs or a templated bridge that substitutes the model name
4. The `fdm_port_in` in the model SDF is per-drone (9002, 9012, ...) — each drone needs its own SDF or the port needs to be overridden at spawn time
