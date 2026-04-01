# Precision Landing — How It Works

## Quick Start (Step by Step)

You need **3 terminals** running in order:

### Terminal 1: Gazebo + ArduPilot SITL
```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py rviz:=true use_gz_tf:=true
```
Wait until Gazebo is fully loaded and the drone model is visible.

### Terminal 2: Precision Landing Stack
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch dbvf_autonomy precision_landing_sim.launch.py
```
This starts 4 nodes. You should see all 4 report started. **Nothing else happens yet** — the system is in IDLE state waiting for a command.

### Terminal 3: MAVProxy — Arm and Take Off
```bash
mavproxy.py --master udpin:0.0.0.0:14550 --console
```
Then in MAVProxy:
```
mode guided
arm throttle
takeoff 10
```
Wait for the drone to reach ~10m altitude and stabilize.

### Terminal 4 (or same as 3): Trigger the Landing
```bash
ros2 service call /dbvf/start_precision_landing dbvf_msgs/srv/StartPrecisionLanding "{target_lat: -35.3632531, target_lon: 149.1657896}"
```
**This is the trigger.** Without this service call, the system sits in IDLE forever.

The target coordinates are the GPS location of the AprilTag pad in `iris_runway.sdf` (tag at Gazebo pose `50.0, 1.0`).

### Terminal 5 (optional): View tag detections on camera
```bash
rqt_image_view /dbvf/debug/tag_image
```
Shows the camera feed with green outlines around detected AprilTags.

#### Calculating tag GPS from Gazebo coordinates

The world origin is in `iris_runway.sdf` under `<spherical_coordinates>`. Tags are placed in Gazebo ENU (X=East, Y=North). Convert to GPS:
```
lat = origin_lat + Y_north / 111000
lon = origin_lon + X_east / (111000 * cos(origin_lat))
```
For the current tag at `(50.0, 1.0)` with origin `(-35.3632621, 149.1652374)`:
- lat = -35.3632621 + 1.0/111000 = **-35.3632531**
- lon = 149.1652374 + 50.0/90546 = **149.1657896**

---

## What Happens After You Trigger

The precision landing node runs a state machine:

```
IDLE  ──(service call)──>  APPROACH  ──(near target)──>  SEARCH  ──(tag seen 5 frames)──>  DESCEND  ──(touchdown)──>  LANDED
                                                            ^                                  |
                                                            └──────(tag lost >4s)──────────────┘
```

### 1. APPROACH (GUIDED mode)
- Drone flies in GUIDED mode to the target GPS coordinates at 8m altitude
- Transitions to SEARCH once within 2m laterally and near 8m altitude

### 2. SEARCH (GUIDED mode, conditional descent)
- Drone holds altitude over the target location until the AprilTag is visible
- Only descends (0.3 m/s) when the tag is actively detected in the camera
- Needs **5 consecutive frames** with a detection to confirm the tag and transition to DESCEND
- If altitude drops below 1m without confirming the tag → ABORT

### 3. DESCEND (LAND mode)
- Switches to ArduPilot LAND mode
- Forwards AprilTag detections as MAVLink `LANDING_TARGET` messages at 20Hz
- ArduPilot's built-in precision landing (PLND) adjusts the descent to center on the tag
- If the tag is lost for >4 seconds → back to SEARCH
- Touchdown detected when: disarmed OR (alt < 0.1m AND vertical speed < 0.1 m/s)

### 4. LANDED / ABORT_LAND
- On success: logs "Landing complete" and returns to IDLE
- On abort (tag lost too long, below min altitude, or 60s timeout): switches to LAND mode and lands wherever it is

---

## What to Watch

### Monitor state transitions
```bash
ros2 topic echo /dbvf/landing_state
```
Shows: `IDLE`, `APPROACH`, `SEARCH`, `DESCEND`, `LANDED`

### Monitor tag detections
```bash
ros2 topic echo /dbvf/tag_status
```
Shows whether the AprilTag is being detected and which tag ID.

### Monitor vehicle telemetry
```bash
ros2 topic echo /dbvf/vehicle_state
```
Shows mode, armed status, GPS position, altitude, velocities.

---

## Common Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| All nodes running, nothing happening | System is in IDLE — no service call sent | Call `/dbvf/start_precision_landing` |
| `sysid=0 compid=0` in mavlink_interface | Connected before SITL sent first heartbeat | Usually resolves after a few seconds; restart node if it persists |
| Drone flies to wrong location | Target GPS coords don't match tag's Gazebo position | Recalculate using ENU→GPS formula above (X→lon, Y→lat — easy to swap) |
| Stuck in SEARCH, never reaches DESCEND | Camera can't see AprilTag from current position | Check drone is over the tag; use `rqt_image_view /dbvf/debug/tag_image` to verify |
| Immediate ABORT | Drone below min search altitude (1m) when entering SEARCH | Take off higher before triggering |
| 60s timeout abort | Overall landing took too long | Check approach coordinates match actual tag location |

---

## Architecture (5 Nodes)

```
[Gazebo Camera] → /camera/image + /camera/camera_info
        ↓
[apriltag_node]         Detects AprilTags in camera frames
        ↓ /apriltag/detections
[tag_detector_adapter]  Picks best tag (big vs small), computes angles, debounces
        ↓ /dbvf/landing_target_pose + /dbvf/tag_status
[precision_landing]     State machine — decides when to approach, search, descend
        ↓ /dbvf/cmd/landing_target + service calls
[mavlink_interface]     Sends MAVLink commands to ArduPilot (mode, position, LANDING_TARGET)

[tag_visualizer]        Draws detection overlays on camera feed → /dbvf/debug/tag_image
```

## Key Parameters (sim_params.yaml)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `approach_altitude` | 8.0m | Altitude for initial approach |
| `min_search_altitude` | 1.0m | Abort if below this during search |
| `search_descent_rate` | 0.3 m/s | How fast to descend while searching |
| `position_tolerance` | 2.0m | Lateral distance to consider "at target" |
| `tag_confirm_frames` | 5 | Consecutive detections needed to confirm tag |
| `tag_lost_timeout` | 4.0s | Seconds without detection before going back to search |
| `landing_timeout` | 60.0s | Total time before aborting |

---

# Mission Sequencer — Full Autonomous Mission

## Quick Start (Step by Step)

You need **5 terminals** running in order:

### Terminal 1: Gazebo + ArduPilot SITL
```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py rviz:=true use_gz_tf:=true
```
Wait until Gazebo is fully loaded and the drone model is visible.

### Terminal 2: Full Mission Stack
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch dbvf_autonomy mission_sim.launch.py
```
This starts 6 nodes (apriltag_node, tag_detector_adapter, mavlink_interface, precision_landing, tag_visualizer, mission_sequencer). **Nothing happens yet** — the mission sequencer is in IDLE state.

### Terminal 3: MAVProxy — Arm the Drone
```bash
mavproxy.py --master udpin:0.0.0.0:14550 --console
```
Then in MAVProxy:
```
mode guided
arm throttle
```
**Do NOT takeoff manually** — the mission sequencer handles takeoff.

### Terminal 4: Monitor Mission Progress
```bash
ros2 topic echo /dbvf/mission_state
```
And optionally in another pane:
```bash
ros2 topic echo /dbvf/mission_phase
```

### Terminal 5: Start the Mission
```bash
ros2 service call /dbvf/start_mission dbvf_msgs/srv/StartMission "{}"
```

---

## Mission Sequence

The mission sequencer runs a 17-state linear state machine covering the full DBVF competition:

```
IDLE → PREFLIGHT_CHECK → TAKEOFF_H → TRANSIT_H_TO_L → LAND_L → WAIT_FLAGGER
       → TAKEOFF_L → TRANSIT_TO_DROP → DROP_PAYLOAD → TRANSIT_TO_WA → LAND_WA
       → TAKEOFF_WA → TRANSIT_TO_DROP_2 → DROP_PAYLOAD_2 → TRANSIT_TO_H → LAND_H → COMPLETE
```

### FM-1: Fly to Landing Zone L
1. **PREFLIGHT_CHECK** — Verifies armed + GUIDED mode
2. **TAKEOFF_H** — Climbs to transit altitude (35ft / ~10.7m)
3. **TRANSIT_H_TO_L** — Flies to landing zone L GPS coordinates
4. **LAND_L** — Switches to LAND mode, waits for touchdown
5. **WAIT_FLAGGER** — Waits on the ground for flagger approval

To resume after the flagger gives the go-ahead:
```bash
ros2 service call /dbvf/resume_mission dbvf_msgs/srv/ResumeMission "{}"
```

### FM-2: Drop Red Payload
6. **TAKEOFF_L** — Re-arms, climbs to transit altitude
7. **TRANSIT_TO_DROP** — Flies to drop zone (F1 or F2, configurable)
8. **DROP_PAYLOAD** — Actuates servo to release payload, waits for settle time

### FM-3: Precision Land at WA, Pickup Yellow, Re-drop
9. **TRANSIT_TO_WA** — Flies to WA (AprilTag landing pad)
10. **LAND_WA** — Triggers precision landing system (AprilTag-guided)
11. **TAKEOFF_WA** — Re-arms, climbs to transit altitude
12. **TRANSIT_TO_DROP_2** — Flies back to drop zone
13. **DROP_PAYLOAD_2** — Releases second payload

### RTH: Return Home
14. **TRANSIT_TO_H** — Flies back to home position
15. **LAND_H** — Switches to LAND mode, lands
16. **COMPLETE** — Mission finished

---

## Abort

To abort at any time:
```bash
ros2 service call /dbvf/abort_mission dbvf_msgs/srv/AbortMission "{reason: 'Manual abort'}"
```

The mission also auto-aborts on:
- **Mission timeout** (540s default, excludes WAIT_FLAGGER time)
- **Heartbeat loss** (5s without ArduPilot heartbeat)
- **Precision landing failure** at WA

On abort, the drone switches to LAND mode and lands in place.

---

## Configuration (mission_params.yaml)

After changing any parameters in `src/dbvf_autonomy/config/mission_params.yaml`, you must rebuild and re-source for changes to take effect:
```bash
colcon build --packages-select dbvf_autonomy && source install/setup.bash
```
The YAML is installed to `install/dbvf_autonomy/share/` during build. The launch file reads from there, not from `src/`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `home_lat` / `home_lon` | -35.3632621 / 149.1652374 | Home position GPS |
| `landing_lat` / `landing_lon` | -35.3632621 / 149.1662471 | Landing zone L GPS |
| `wa_lat` / `wa_lon` | -35.3633033 / 149.1657423 | WA (AprilTag pad) GPS |
| `f1_lat` / `f1_lon` | -35.3632621 / 149.1665841 | Drop zone F1 GPS |
| `f2_lat` / `f2_lon` | -35.3632621 / 149.1669210 | Drop zone F2 GPS |
| `transit_altitude_ft` | 35.0 | Cruise altitude in feet |
| `position_tolerance_m` | 3.0 | Lateral tolerance for waypoint arrival |
| `takeoff_complete_alt_ft` | 33.0 | Altitude to consider takeoff complete |
| `drop_servo_number` | 9 | Servo channel for payload release |
| `drop_servo_pwm_release` | 1100 | PWM value to release payload |
| `drop_servo_pwm_hold` | 1500 | PWM value to hold payload |
| `drop_settle_time_s` | 2.0 | Time to wait after servo actuation |
| `drop_target` | "F1" | Which drop zone to use ("F1" or "F2") |
| `mission_timeout_s` | 540.0 | Total mission timeout (9 minutes) |
| `prefer_rangefinder` | true | Use rangefinder altitude if available |

---

## What to Watch

### Monitor mission state and phase
```bash
ros2 topic echo /dbvf/mission_state    # Current FSM state (e.g., TRANSIT_H_TO_L)
ros2 topic echo /dbvf/mission_phase    # Current phase (FM1, FM2, FM3, RTH)
```

### All DBVF topics
```bash
ros2 topic list | grep dbvf
```
