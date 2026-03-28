# Rangefinder Integration — Design Spec

**Date:** 2026-03-28
**Status:** Approved
**Scope:** Add simulated downward ToF rangefinder to Gazebo, feed data to ArduPilot SITL and ROS2

---

## Goal

Integrate a simulated rangefinder into the existing Gazebo simulation that matches the ARK Flow's Broadcom AFBR-S50LV85D Time-of-Flight sensor. The rangefinder data must be available to both ArduPilot SITL (for EKF altitude fusion and PLND) and as a ROS2 topic (for custom landing logic in dbvf_autonomy). This is the foundation for a follow-on landing logic rework that will use rangefinder data for descent rate limiting and altitude floor enforcement.

---

## Hardware Reference

**Sensor:** ARK Flow — Broadcom AFBR-S50LV85D ToF rangefinder
- Range: 0.1m – 30m
- Field of view: 12.4° x 6.2° (32 pixels)
- Transmitter beam: 2° x 2°
- Interface: DroneCAN (UAVCAN) on real hardware

For simulation purposes, a single-beam downward `gpu_lidar` sensor approximates the ToF behavior. The narrow beam and flat landing surface mean multi-pixel FOV is unnecessary.

---

## Architecture

```
Gazebo gpu_lidar sensor (on drone, pointing down)
        │
        ├──► ArduPilotPlugin <sensor> block
        │         │
        │         └──► ArduPilot SITL (rng_1 in JSON state)
        │                   │
        │                   ├──► EKF altitude fusion
        │                   ├──► PLND altitude gating
        │                   └──► RANGEFINDER MAVLink message
        │                              │
        │                              └──► mavlink_interface_node
        │                                        │
        │                                        └──► VehicleState.range_alt
        │
        └──► ros_gz_bridge
                  │
                  └──► /rangefinder (sensor_msgs/LaserScan)
                            │
                            └──► Available for direct consumption / debugging
```

Two parallel data paths:
1. **Gazebo → ArduPilotPlugin → SITL → MAVLink → mavlink_interface_node → VehicleState** — integrated into existing node architecture
2. **Gazebo → ros_gz_bridge → /rangefinder topic** — raw sensor data for debugging or direct consumption

---

## Component Details

### 1. Gazebo Sensor (iris_with_standoffs model SDF)

Add a `rangefinder_link` with a fixed joint to `base_link`, oriented to point straight down (-90° pitch).

Sensor specification:
- **Type:** `gpu_lidar`
- **Samples:** 1 horizontal, 1 vertical (single beam)
- **Range:** 0.1m min, 30m max, 0.01m resolution
- **Update rate:** 50 Hz
- **Gazebo topic:** `/rangefinder`
- **Noise:** Gaussian, mean 0.0, stddev 0.01m
- **Visualize:** true (for debugging)

The sensor is placed on `iris_with_standoffs` (base drone model) rather than `iris_with_gimbal` so it's available to any model variant that includes the base iris.

### 2. ArduPilotPlugin Configuration (iris_with_gimbal model SDF)

Add a `<sensor>` block inside the existing `<plugin name="ArduPilotPlugin">` element:

```xml
<sensor>
  <type>lidar</type>
  <index>1</index>
  <topic>/rangefinder</topic>
</sensor>
```

The plugin subscribes to the Gazebo LaserScan topic, extracts the minimum range value, and sends it as `rng_1` in the JSON state packet to ArduPilot SITL via UDP.

### 3. ArduPilot Parameters (gazebo-iris-hardmount.parm)

```
RNGFND1_TYPE     100
RNGFND1_MIN_CM   10
RNGFND1_MAX_CM   3000
RNGFND1_ORIENT   25
```

- `TYPE=100` — SITL rangefinder (reads from sim backend JSON)
- `MIN_CM=10` — 0.1m minimum range (matches Broadcom ToF)
- `MAX_CM=3000` — 30m maximum range (matches Broadcom ToF)
- `ORIENT=25` — Downward-facing

### 4. ROS2 Bridge (iris_bridge.yaml)

Add one entry:

```yaml
- ros_topic_name: "rangefinder"
  gz_topic_name: "/rangefinder"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
```

Publishes at sensor rate (50 Hz). For single-beam rangefinder, consumers read `msg.ranges[0]`.

### 5. VehicleState Message (VehicleState.msg)

Add one field:

```
float64 range_alt    # Rangefinder AGL altitude (meters), -1.0 if no valid reading
```

Sentinel value `-1.0` indicates no rangefinder data received yet.

### 6. MAVLink Interface Node (mavlink_interface_node.py)

Parse the `RANGEFINDER` MAVLink message from ArduPilot:
- Extract `distance` field (meters)
- Store as `self.range_alt` (initialized to `-1.0`)
- Populate `VehicleState.range_alt` in the publish loop

The RANGEFINDER message is emitted by ArduPilot at ~10 Hz when a rangefinder is configured. No request needed — it streams automatically.

---

## Files Modified

| File | Change |
|------|--------|
| `src/ardupilot_gazebo/models/iris_with_standoffs/model.sdf` | Add `rangefinder_link` with fixed joint + `gpu_lidar` sensor |
| `src/ardupilot_gazebo/models/iris_with_gimbal/model.sdf` | Add `<sensor>` block to ArduPilotPlugin config |
| `src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm` | Add 4 RNGFND1 parameters |
| `src/ardupilot_gz/ardupilot_gz_bringup/config/iris_bridge.yaml` | Add rangefinder bridge entry |
| `src/dbvf_msgs/msg/VehicleState.msg` | Add `float64 range_alt` field |
| `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py` | Parse RANGEFINDER MAVLink message, populate range_alt |
| `src/dbvf_autonomy/test/test_mavlink_messages.py` | Add rangefinder-related unit tests |

## Files NOT Modified

- `precision_landing_node.py` — no landing logic changes (deferred)
- `tag_detector_adapter_node.py` — no tag switching changes (deferred)
- Launch files — no new nodes added

---

## Testing

### Unit Tests (added to test_mavlink_messages.py)

- `test_range_alt_sentinel_before_data`: range_alt is -1.0 before first RANGEFINDER message received
- `test_range_alt_populated_from_rangefinder`: range_alt populated correctly from RANGEFINDER MAVLink message distance field

### Integration Verification (manual)

1. Launch `iris_runway.launch.py` — confirm rangefinder ray visible in Gazebo
2. `ros2 topic echo /rangefinder` — confirm LaserScan messages with reasonable range values
3. `ros2 topic echo /dbvf/vehicle_state` — confirm `range_alt` populated and tracks drone altitude
4. MAVProxy: `status rangefinder1` — confirm ArduPilot sees the rangefinder
5. Fly drone up/down — verify range values track AGL altitude correctly

---

## Out of Scope (Deferred to Landing Logic Rework)

- Dual-tag switching logic (both tags visible for 2 consecutive seconds)
- Hard altitude floor enforcement using rangefinder
- Descent rate limiting based on rangefinder altitude
- Moving from ArduPilot LAND mode to custom GUIDED-mode descent
- Optical flow simulation (ARK Flow also has PAW3902, not needed now)
