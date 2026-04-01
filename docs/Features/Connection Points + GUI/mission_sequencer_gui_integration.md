# Mission Sequencer — GUI Integration Guide

This document describes the ROS2 topics, services, and configurable GPS parameters the GUI team needs to integrate with the mission sequencer.

---

## GPS Waypoints That Must Be Set Per-Mission

These parameters are in `config/mission_params.yaml` under `mission_sequencer.ros__parameters`. **The GUI must update these to match the actual competition field layout before each mission.**

| Parameter | Description | Used By States |
|-----------|-------------|----------------|
| `home_lat` / `home_lon` | Home position (takeoff point H) | TAKEOFF_H, TRANSIT_TO_H, LAND_H |
| `landing_lat` / `landing_lon` | Landing zone L (FM-1 land target) | TRANSIT_H_TO_L, LAND_L, TAKEOFF_L |
| `wa_lat` / `wa_lon` | Work Area (AprilTag precision landing pad) | TRANSIT_TO_WA, LAND_WA, TAKEOFF_WA |
| `f1_lat` / `f1_lon` | Drop zone F1 | TRANSIT_TO_DROP, DROP_PAYLOAD, TRANSIT_TO_DROP_2, DROP_PAYLOAD_2 |
| `f2_lat` / `f2_lon` | Drop zone F2 (alternate) | Same as F1 (selected via `drop_target`) |

### Other Configurable Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `drop_target` | `"F1"` | Which drop zone to use — `"F1"` or `"F2"` |
| `transit_altitude_ft` | `35.0` | Cruise altitude in feet |
| `position_tolerance_m` | `3.0` | How close (meters) to a waypoint before considering "arrived" |
| `takeoff_complete_alt_ft` | `33.0` | Altitude (ft) to consider takeoff complete |
| `drop_servo_number` | `9` | Servo channel for payload release mechanism |
| `drop_servo_pwm_release` | `1100` | PWM to release payload |
| `drop_servo_pwm_hold` | `1500` | PWM to hold payload |
| `drop_settle_time_s` | `2.0` | Seconds to wait after servo actuation |
| `mission_timeout_s` | `540.0` | Total mission timeout (9 min), excludes WAIT_FLAGGER |
| `prefer_rangefinder` | `true` | Use rangefinder for altitude if available |

### How to Set Parameters

**Option A — Edit YAML before launch:**
Edit `src/dbvf_autonomy/config/mission_params.yaml`, rebuild, then launch.

**Option B — Override at launch time:**
```bash
ros2 launch dbvf_autonomy mission_sim.launch.py \
  --ros-args -p home_lat:=-35.3632621 -p home_lon:=149.1652374
```

**Option C — Set via ROS2 param (if GUI sets params programmatically):**
```bash
ros2 param set /mission_sequencer home_lat -35.3632621
```
Note: Parameters are read at node startup. Changing them after launch only takes effect if the mission is restarted.

---

## Topics for GUI Feedback

### `/dbvf/mission_state` (`std_msgs/String`)

Published at **10 Hz**. Contains the current FSM state as a string. The GUI should subscribe to this for detailed state display.

**All possible values:**

| State String | Description | What the GUI Should Show |
|-------------|-------------|--------------------------|
| `IDLE` | Waiting for start command | "Ready" / waiting indicator |
| `PREFLIGHT_CHECK` | Verifying armed + GUIDED mode | "Preflight..." |
| `TAKEOFF_H` | Climbing to transit altitude at Home | "Taking off from Home" |
| `TRANSIT_H_TO_L` | Flying from Home to Landing Zone L | "Flying to L" + progress |
| `LAND_L` | Descending at L | "Landing at L" |
| `WAIT_FLAGGER` | On ground at L, waiting for flagger | "Waiting for flagger" (highlight — needs human action) |
| `TAKEOFF_L` | Climbing from L | "Taking off from L" |
| `TRANSIT_TO_DROP` | Flying to drop zone | "Flying to drop zone" |
| `DROP_PAYLOAD` | Servo actuated, payload releasing | "Dropping payload 1" |
| `TRANSIT_TO_WA` | Flying to Work Area | "Flying to WA" |
| `LAND_WA` | Precision landing on AprilTag at WA | "Precision landing at WA" |
| `TAKEOFF_WA` | Climbing from WA | "Taking off from WA" |
| `TRANSIT_TO_DROP_2` | Flying to drop zone (second run) | "Flying to drop zone (2)" |
| `DROP_PAYLOAD_2` | Releasing second payload | "Dropping payload 2" |
| `TRANSIT_TO_H` | Flying back to Home | "Returning home" |
| `LAND_H` | Landing at Home | "Landing at Home" |
| `COMPLETE` | Mission finished | "Mission complete" (success indicator) |
| `ABORT` | Mission aborted | "ABORTED" (error indicator, red) |

### `/dbvf/mission_phase` (`std_msgs/String`)

Published **on phase change only** (not every tick). Higher-level grouping for simplified display.

| Phase String | Description | States Included |
|-------------|-------------|-----------------|
| `IDLE` | Not started | IDLE |
| `PREFLIGHT` | Preflight checks | PREFLIGHT_CHECK |
| `FM1` | Flight Mission 1 — fly to L and land | TAKEOFF_H, TRANSIT_H_TO_L, LAND_L, WAIT_FLAGGER |
| `FM2` | Flight Mission 2 — drop red payload | TAKEOFF_L, TRANSIT_TO_DROP, DROP_PAYLOAD |
| `FM3` | Flight Mission 3 — precision land, pickup, re-drop | TRANSIT_TO_WA, LAND_WA, TAKEOFF_WA, TRANSIT_TO_DROP_2, DROP_PAYLOAD_2 |
| `RTH` | Return to Home | TRANSIT_TO_H, LAND_H |
| `COMPLETE` | Mission finished | COMPLETE |
| `ABORT` | Mission aborted | ABORT |

### `/dbvf/vehicle_state` (`dbvf_msgs/VehicleState`)

Published at **10 Hz** by mavlink_interface_node. Useful for live telemetry display.

```
string mode        # ArduPilot mode (GUIDED, LAND, etc.)
bool armed
float64 lat        # Current GPS latitude
float64 lon        # Current GPS longitude
float64 alt_rel    # Relative altitude (meters)
float64 vx, vy, vz # Velocity (m/s)
float64 heading    # Degrees
float64 range_alt  # Rangefinder altitude (-1.0 if unavailable)
```

### `/dbvf/altitude_source` (`std_msgs/String`) — *planned, not yet implemented*

Published at **10 Hz** by mission_sequencer_node. Indicates which altitude sensor the FSM is currently trusting for altitude decisions.

| Value | Meaning |
|-------|---------|
| `"rangefinder"` | Rangefinder reading is valid (>= 0.0 and <= `rangefinder_max_m`) and being used |
| `"barometer"` | Falling back to barometric `alt_rel` (rangefinder unavailable or out of range) |

The GUI can combine this with the raw values from `/dbvf/vehicle_state` (`range_alt` and `alt_rel`) to show both readings and highlight which one is active. Useful for diagnosing altitude sensor issues in flight.

### `/dbvf/heartbeat_status` (`std_msgs/Bool`)

Published at **1 Hz**. `true` if ArduPilot heartbeat received within last 3 seconds. GUI can show a connection indicator.

### `/dbvf/landing_state` (`std_msgs/String`)

Published at **20 Hz** during precision landing. Values: `IDLE`, `APPROACH`, `SEARCH`, `DESCEND`, `LANDED`, `ABORT_LAND`. Useful for detailed WA landing progress during FM-3.

### `/dbvf/tag_status` (`dbvf_msgs/TagStatus`)

Published at camera rate. Shows AprilTag detection status — useful during LAND_WA state.

```
bool detected
int32 active_tag_id    # -1 if none
int32 frames_since_last
float64 confidence     # 0.0-1.0
```

---

## Services the GUI Can Call

### `/dbvf/start_mission` (`dbvf_msgs/srv/StartMission`)

Start the autonomous mission. Only works when state is `IDLE`.

```bash
ros2 service call /dbvf/start_mission dbvf_msgs/srv/StartMission "{}"
```

Response: `bool success, string message`

### `/dbvf/resume_mission` (`dbvf_msgs/srv/ResumeMission`)

Resume from WAIT_FLAGGER (after flagger gives go-ahead). Only works when state is `WAIT_FLAGGER`. **Automatically sets GUIDED mode, arms the throttle, and commands takeoff** — no manual MAVProxy steps required. This is the only button the team needs to press to continue the mission after the flagger approves.

```bash
ros2 service call /dbvf/resume_mission dbvf_msgs/srv/ResumeMission "{}"
```

Response: `bool success, string message`

### `/dbvf/abort_mission` (`dbvf_msgs/srv/AbortMission`)

Emergency abort from any state. Drone switches to LAND mode immediately.

```bash
ros2 service call /dbvf/abort_mission dbvf_msgs/srv/AbortMission "{reason: 'Operator abort'}"
```

Response: `bool success, string message`

---

## Suggested GUI Layout

```
+--------------------------------------------------+
|  DBVF Mission Control                     [HB: OK]|
+--------------------------------------------------+
|  Phase: FM2           State: TRANSIT_TO_DROP      |
|  Mode:  GUIDED        Armed: YES                  |
|  Alt:   10.7m (35ft)  Src: rangefinder            |
|  Rng:   10.7m   Baro: 10.9m                      |
|  GPS: -35.3645, 149.1652                          |
+--------------------------------------------------+
|  [Start Mission]  [Resume]  [ABORT]               |
+--------------------------------------------------+
|  Waypoints:                                       |
|    Home (H):  -35.3632621, 149.1652374            |
|    Land (L):  -35.3640000, 149.1652374            |
|    Drop (F1): -35.3650000, 149.1652374            |
|    WA:        -35.3632531, 149.1657896            |
+--------------------------------------------------+
```

- **Start Mission** button: calls `/dbvf/start_mission` (disabled unless IDLE)
- **Resume** button: calls `/dbvf/resume_mission` (disabled unless WAIT_FLAGGER)
- **ABORT** button: calls `/dbvf/abort_mission` (always enabled, red, prominent)
- **HB indicator**: green if `/dbvf/heartbeat_status` is true, red otherwise
- **Phase/State**: from `/dbvf/mission_phase` and `/dbvf/mission_state`
- **Telemetry**: from `/dbvf/vehicle_state`
