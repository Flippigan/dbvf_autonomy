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
