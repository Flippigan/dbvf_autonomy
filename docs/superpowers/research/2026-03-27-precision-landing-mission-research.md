# Precision Landing & Autonomous Mission Research

**Date:** 2026-03-27
**Status:** Research Complete
**Scope:** Best methods for precision landing, takeoff, drop zone detection, and autonomous mission scripting for VFS DBVF 2025-2026 competition on NVIDIA Jetson Orin

---

## Table of Contents

1. [AprilTag Precision Landing on ArduPilot](#1-apriltag-precision-landing-on-ardupilot)
2. [Jetson Orin Computer Vision Pipeline](#2-jetson-orin-computer-vision-pipeline)
3. [Drop Zone Detection & Payload Centering](#3-drop-zone-detection--payload-centering)
4. [Autonomous Mission Scripting](#4-autonomous-mission-scripting)
5. [Payload Drop Precision Optimization](#5-payload-drop-precision-optimization)
6. [Recommended System Architecture](#6-recommended-system-architecture)

---

## 1. AprilTag Precision Landing on ArduPilot

### 1.1 ArduPilot Precision Landing Parameters

#### Core Parameters

| Parameter | Recommended Value | Description |
|-----------|------------------|-------------|
| `PLND_ENABLED` | 1 | Enables precision landing. Reboot required to expose additional params. |
| `PLND_TYPE` | 1 | Companion computer mode (MAVLink LANDING_TARGET). Options: 0=None, 1=Companion, 2=IRLock, 3=SITL_Gazebo, 4=SITL. |
| `PLND_EST_TYPE` | 0 (start here) | 0=RawSensor, 1=KalmanFilter. The Kalman filter has caused instability in some setups (GitHub issue #6614). Start with 0 (raw) and test KF only after everything works. |
| `PLND_LAG` | 0.02-0.250 (seconds) | Compensates for camera/processing pipeline latency. Measure your actual latency and set accordingly. Incorrect values cause the system to correct in the wrong direction. |
| `PLND_ORIENT` | 25 | Camera mounting orientation. 25 = Down (for downward-facing camera). |
| `PLND_YAW_ALIGN` | 0 | Yaw rotation from body X-axis to sensor X-axis, in centidegrees (0-36000). Set to 0 if camera X-axis aligns with vehicle forward. |

#### Altitude and Distance Gating

| Parameter | Recommended Value | Description |
|-----------|------------------|-------------|
| `PLND_ALT_MAX` | 8-10 (meters) | If target not found above this altitude, vehicle descends vertically until this height, then retry logic kicks in. Set based on large tag's max detection range. |
| `PLND_ALT_MIN` | 0.5-1.0 (meters) | If target lost below this height, continue vertical descent (don't retry). |
| `PLND_XY_DIST_MAX` | 3-5 (meters) | Max lateral distance from target before descent stops. 0 = always descend regardless of lateral offset. |

#### Retry Behavior

| Parameter | Recommended Value | Description |
|-----------|------------------|-------------|
| `PLND_STRICT` | 1 | 0=Land vertically if lost, 1=Retry then land, 2=Retry then hover. Use 1 for competition. |
| `PLND_RET_MAX` | 3 | Maximum retry attempts. |
| `PLND_TIMEOUT` | 4 (seconds) | Time descending after target loss before retry triggers. |
| `PLND_RET_BEHAVE` | 0 | 0=Go to last seen target position, 1=Go to approximate target position. |

#### Camera Position Offsets

| Parameter | Description |
|-----------|-------------|
| `PLND_CAM_POS_X` | Camera X position in body frame (forward positive), meters |
| `PLND_CAM_POS_Y` | Camera Y position in body frame (right positive), meters |
| `PLND_CAM_POS_Z` | Camera Z position in body frame (down positive), meters |

#### Additional Required Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `SERIALx_PROTOCOL` | 2 | MAVLink2 on the UART connected to companion computer |
| `SERIALx_BAUD` | 921600 | Match baud rate with companion computer |
| `RNGFND1_TYPE` | (your rangefinder) | A downward rangefinder is strongly recommended for accurate AGL altitude |
| `RNGFND1_ORIENT` | 25 (Down) | Orientation for the rangefinder |

### 1.2 MAVLink LANDING_TARGET Message

#### Message Fields

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| `time_usec` | uint64_t | us | Timestamp since system boot |
| `target_num` | uint8_t | -- | Target ID (use 0) |
| `frame` | uint8_t | MAV_FRAME | Use `MAV_FRAME_BODY_FRD` (12) |
| `angle_x` | float | rad | X-axis angular offset from camera center to target |
| `angle_y` | float | rad | Y-axis angular offset from camera center to target |
| `distance` | float | m | Distance to target (set 0 if using rangefinder) |
| `size_x` | float | rad | Angular size of target along x-axis |
| `size_y` | float | rad | Angular size of target along y-axis |
| `x` | float | m | X position in body frame (forward) -- MAVLink 2 extension |
| `y` | float | m | Y position in body frame (right) -- MAVLink 2 extension |
| `z` | float | m | Z position in body frame (down) -- MAVLink 2 extension |
| `q` | float[4] | -- | Quaternion [w,x,y,z] of target orientation |
| `type` | uint8_t | -- | `LANDING_TARGET_TYPE_VISION_FIDUCIAL` (3) |
| `position_valid` | uint8_t | -- | 0 = use angle_x/angle_y only, 1 = use x/y/z position |

#### Two Approaches

**Angles-only (simpler, recommended to start):**
```python
angle_x = math.atan2((u - cx) / fx, 1.0)
angle_y = math.atan2((v - cy) / fy, 1.0)
```
Set `position_valid = 0`, `distance = 0` if you have a rangefinder.

**Full pose (more accurate):**
```python
# From solvePnP translation vector (tvec):
x_body = tvec[2]   # camera Z = body forward
y_body = tvec[0]   # camera X = body right
z_body = tvec[1]   # camera Y = body down
```
Set `position_valid = 1`.

#### Message Rate
- Minimum: 1 Hz (ArduPilot documentation minimum)
- Recommended: **20-30 Hz** while target is visible
- Higher rates reduce oscillation and improve tracking

#### pymavlink Example
```python
m.mav.landing_target_send(
    int(time.time() * 1e6),               # time_usec
    0,                                      # target_num
    mavutil.mavlink.MAV_FRAME_BODY_NED,    # frame
    float(angle_x),                         # angle_x (rad)
    float(angle_y),                         # angle_y (rad)
    0.0,                                    # distance (0 if using rangefinder)
    TAG_SIZE_M, TAG_SIZE_M,                # size_x, size_y
    x_body, y_body, z_body,                # body frame position
    [1.0, 0.0, 0.0, 0.0],                 # quaternion (identity)
    mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
    1 if use_position else 0               # position_valid
)
```

### 1.3 AprilTag Detection Library Comparison

| Library | GPU Accel | ROS2 Native | FPS (Jetson Orin) | Notes |
|---------|-----------|-------------|-------------------|-------|
| **isaac_ros_apriltag** | CUDA + PVA | Yes (Humble) | **116-178 fps** @ 720p | Best performance, NVIDIA official |
| **nvapriltags_ros2** (Kiwicampus) | CUDA | Yes (Foxy/Humble) | ~30-50 fps | Lighter weight than full Isaac ROS |
| **apriltag_ros** (christianrauch) | CPU only | Yes (Humble) | ~6-12 fps | Standard ROS2 package, too slow |
| **pupil-apriltags** | CPU only | No (Python) | ~5-26 fps | Easy Python API, too slow for real-time |

**Recommendation:** Use **isaac_ros_apriltag** with CUDA backend for tag36h11 at 120+ fps.

### 1.4 Dual-Tag Switching Logic

For the two AprilTags (0.6m primary ID 0, 0.15m secondary ID 1):

| Altitude | Active Tag | Rationale |
|----------|-----------|-----------|
| 6.1m - ~2m | Primary (0.6m, ID 0) | Large tag clearly visible, good pixel count |
| ~2m - ~1.5m | Transition zone | Both tags visible; prefer primary until it fills FOV |
| ~1.5m | Switch to Secondary (0.15m, ID 1) | Primary fills/exceeds FOV; secondary becomes detectable |
| <1.5m - ground | Secondary (0.15m, ID 1) | Fine precision for final touchdown |

**Switching rules:**
1. Always prefer the largest detected tag (best pose estimation)
2. When the large tag exceeds FOV, the detector naturally fails to detect it
3. Debounce switching: maintain 30-frame history buffer (~1 second), require 80% detection consistency before locking onto the smaller tag
4. If neither tag is detected, stop sending LANDING_TARGET and let ArduPilot's PLND_STRICT logic handle target loss

### 1.5 ArduPilot Precision Landing Behavior

**Critical finding:** Precision landing does NOT work reliably in AUTO mode with NAV_LAND. Multiple users confirm this.

**Recommended mission sequence:**
1. **AUTO mode**: Fly mission waypoints to approach WA at transit altitude (30 ft AGL)
2. **GUIDED mode**: Navigate to GPS coordinates of WA center at transit altitude
3. **Begin descent in GUIDED**: Descend to `PLND_ALT_MAX` altitude where tag detection begins
4. **Switch to LAND mode**: Once companion computer confirms tag detection, command LAND mode
5. ArduPilot's precision landing takes over, guiding the vehicle to the tag

**MAVROS warning:** The MAVROS landing_target plugin does NOT work reliably with ArduPilot precision landing. Use **pymavlink directly** over serial UART for LANDING_TARGET messages.

### 1.6 Landing Accuracy

| Condition | Expected Accuracy |
|-----------|-------------------|
| Best case (well-tuned, calm) | 5-15 cm |
| Typical first attempt | 30-50 cm |
| Without rangefinder | >1 m (significantly degraded) |
| With rangefinder + good tuning | 10-20 cm |

**Error sources (ranked):**
1. Camera latency (100ms delay at 0.5 m/s = 5cm error)
2. Camera calibration
3. Tag pose estimation noise
4. Wind and vehicle dynamics
5. `PLND_LAG` misconfiguration
6. Vehicle tune quality (PID, position controller)
7. Rolling shutter artifacts

### 1.7 Known Issues and Pitfalls

1. **Precision Landing does NOT work in AUTO mode NAV_LAND** -- use LOITER then switch to LAND
2. **MAVROS Landing Target Plugin unreliable** -- use pymavlink directly
3. **Kalman Filter instability** (`PLND_EST_TYPE=1`) -- start with raw sensor (0)
4. **Jerky corrections pre-4.2** -- use Copter 4.5+ for smooth PosControl trajectories
5. **Inverted angles** -- verify PLND_ORIENT and PLND_YAW_ALIGN match camera mounting
6. **Missing rangefinder** -- altitude gating doesn't work properly without one
7. **Low detection rate causes oscillation** -- need 20+ Hz minimum
8. **Camera motion blur** -- minimize exposure time, prefer global shutter cameras
9. **Large tag filling FOV** -- dual-tag approach solves this
10. **Wind-induced oscillation** -- reduce LAND_SPEED in windy conditions

### Sources
- [Precision Landing and Loiter -- Copter documentation](https://ardupilot.org/copter/docs/precision-landing-and-loiter.html)
- [Precision Landing -- Dev documentation](https://ardupilot.org/dev/docs/mavlink-precision-landing.html)
- [Landing Target Protocol -- MAVLink Guide](https://mavlink.io/en/services/landing_target.html)
- [Precision Landing Kalman Filter instability -- GitHub Issue #6614](https://github.com/ArduPilot/ardupilot/issues/6614)
- [Precision Land not working in AUTO Mode -- ArduPilot Discourse](https://discuss.ardupilot.org/t/precision-land-not-working-in-auto-mode/88587)
- [vision-landing-2 -- GitHub (RosettaDrone)](https://github.com/RosettaDrone/vision-landing-2)
- [Precision Drone Landing with Visual and IR Fiducial Markers -- ArXiv 2403.03806](https://arxiv.org/abs/2403.03806)

---

## 2. Jetson Orin Computer Vision Pipeline

### 2.1 NVIDIA Isaac ROS AprilTag

**Performance benchmarks (720p input, Isaac ROS 3.2):**

| Platform | FPS | Latency |
|----------|-----|---------|
| AGX Orin | 178 fps | 6.3 ms |
| Orin NX | 116 fps | 9.4 ms |
| Orin Nano Super 8GB | 123 fps | 8.6 ms |

- CUDA backend supports only **tag36h11** (exactly what we need)
- CPU and PVA backends support all 9 tag families
- ROS2 Humble compatible via Isaac ROS 3.2
- Requires JetPack 6+, Docker container workflow recommended
- ~20x faster than CPU-based apriltag_ros (~6 fps)

### 2.2 Camera Interface Options

| Camera | Sensor | Shutter | Notes |
|--------|--------|---------|-------|
| **IMX477 (HQ Camera)** | Sony IMX477 | Rolling | Most popular, excellent low-light, ~$25-40 |
| **IMX219** | Sony IMX219 | Rolling | Cheap but lower sensitivity |
| **OV2311** | OmniVision | Global | Eliminates motion blur, 2MP |
| **AR0234** | ON Semi | Global | Good for moving platforms, 2.3MP |

**Recommendation:** IMX477 with wide-angle lens (CSI interface) for pragmatic choice. Consider OV2311/AR0234 global shutter if motion blur is a problem in testing.

**CSI advantages on Jetson:**
- Raw sensor data goes directly to hardware ISP
- Images land in GPU memory (NVMM) without CPU copies
- Auto-exposure, auto-white-balance handled in hardware
- Lowest possible latency path to GPU processing

### 2.3 Camera Nodes

1. **isaac_ros_argus_camera** (Recommended): NVIDIA's CSI camera node delivering frames directly in GPU memory (NVMM). Zero-copy into NITROS-accelerated nodes.
2. **gscam2**: GStreamer-based, works with CSI and USB. More flexible but less optimized.
3. **v4l2_camera**: Standard ROS2 driver. CPU-based, no GPU acceleration.

### 2.4 Dual Processing Pipeline Architecture

```
                    isaac_ros_argus_camera
                           |
                     [GPU: NVMM Image]
                           |
                    +--------------+
                    |              |
            isaac_ros_apriltag   drop_zone_detector
            (GPU/CUDA backend)   (CPU: OpenCV or GPU: YOLO)
                    |              |
            [AprilTag Poses]  [Zone Detections]
                    |              |
                    +--------------+
                           |
                   mission_controller
                           |
                   [MAVLink LANDING_TARGET]
```

- AprilTag detection runs on **GPU (CUDA)** via isaac_ros_apriltag
- Drop zone detection runs on **CPU** using OpenCV HSV segmentation + contour detection
- These use different hardware resources and do not compete
- All NITROS nodes in same component container for zero-copy GPU memory sharing

### 2.5 Latency Budget

| Pipeline Stage | Estimated Latency |
|---------------|-------------------|
| Camera exposure + readout | 5-15 ms |
| ISP processing (hardware) | 1-3 ms |
| Image rectification (GPU) | 1-2 ms |
| AprilTag detection (GPU) | 8-11 ms |
| Pose calculation + message | 1-2 ms |
| MAVLink transmission | 1-2 ms |
| ArduPilot EKF processing | 5-10 ms |
| **Total end-to-end** | **~22-45 ms** |

This is well within the "excellent" range (<50ms) for precision landing.

### 2.6 Hardware Recommendations

**Compute module:**
- **Orin Nano Super 8GB at 15W**: Best price/performance for competition drones (67 TOPS, ~8g module)
- **Orin NX 16GB at 15-25W**: Better if running additional workloads (YOLO, visual odometry)

**Carrier boards for drones:**
- **Seeed Studio reComputer Mini**: 80g, 88x56x17mm, XT30 power (12-54V DC)
- **Auvidea JNX110**: Pixhawk-integrated, designed for drone platforms

**Software compatibility:**

| Component | Recommended Version |
|-----------|-------------------|
| JetPack | 6.1 or 6.2 (L4T 36.4.3) |
| Ubuntu | 22.04 |
| ROS 2 | Humble (matching existing workspace) |
| Isaac ROS | 3.2 (last Humble-compatible release) |
| CUDA | 12.6 (bundled with JetPack 6.2) |

### 2.7 Camera Calibration

Camera calibration is **critical** for accurate AprilTag pose estimation.

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 \
  image:=/camera/image_raw camera:=/camera
```

Best practices:
- Large checkerboard (8x6+ inner corners) on rigid material
- 30-50 images from diverse angles/distances
- Verify reprojection error < 0.5 pixels
- Re-calibrate after lens changes or physical impacts
- Wide-angle lenses (114+ HFOV) need GPU-accelerated rectification before AprilTag detection

### Sources
- [Isaac ROS AprilTag Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_apriltag/index.html)
- [Isaac ROS Performance Summary](https://nvidia-isaac-ros.github.io/performance/index.html)
- [Isaac ROS Argus Camera](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_argus_camera)
- [nvapriltags_ros2 -- GitHub (Kiwicampus)](https://github.com/kiwicampus/nvapriltags_ros2)
- [Jetson Orin Nano Super Developer Kit](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)

---

## 3. Drop Zone Detection & Payload Centering

### 3.1 Drop Zone Specifications (from VFS DBVF RFP)

| Waypoint | Size | Marking | Points per Payload |
|----------|------|---------|-------------------|
| L, H | 15 x 15 ft | Stakes (<=12" tall) + see-through tarp walls | N/A (landing zones) |
| F1 | 7 x 7 ft | Stakes + see-through tarp walls | 2.5 pts |
| F2 | 3 x 3 ft | Stakes + see-through tarp walls | 5.0 pts |
| WA | 20 x 20 ft | Marked square | N/A (pickup zone) |
| WM | 20 x 20 ft | Marked square | N/A (manual pickup) |

### 3.2 Classical CV Detection Pipeline

**Multi-strategy approach (recommended):**

1. **Preprocessing:** CLAHE (Contrast Limited Adaptive Histogram Equalization) + HSV color space conversion + Gaussian blur (7x7) to suppress grass texture
2. **Edge detection:** Canny (thresholds ~50-150)
3. **Line detection:** Probabilistic Hough Transform (`cv2.HoughLinesP`) with `minLineLength` filter
4. **Rectangle assembly:** Group perpendicular line pairs, find intersections, validate 4-corner rectangles
5. **Size classification:** Convert pixel dimensions to feet using GSD at known altitude

### 3.3 Alternative: Stake/Corner Detection

Stakes (solid physical objects <=12" tall) may be more reliably detected than transparent wrap:
- `cv2.SimpleBlobDetector` for small stake-top blobs
- Look for clusters of 4 blobs forming rectangular patterns
- Harris corner detector for intensity variation points

**If rules permit:** Adding bright-colored caps or tape to stakes would dramatically simplify detection.

### 3.4 Deep Learning Approach (YOLO)

| Model | Precision | Platform | FPS |
|-------|-----------|----------|-----|
| YOLOv8n | FP16 | Orin Nano | ~33 fps |
| YOLOv8n | INT8 | Orin Nano | ~43 fps |
| YOLOv8n | FP16 | Orin NX | ~90 fps |
| YOLO11n | FP16 | Orin (full) | >60 fps |

**Training data strategy:**
- Generate 5,000-10,000 synthetic images from Gazebo simulation at various altitudes and lighting
- Augment with 200-500 real field images
- Fine-tune on real data to close sim-to-real gap
- Train 4 classes: `zone_20ft`, `zone_15ft`, `zone_7ft`, `zone_3ft`

**Hybrid approach:** YOLO for initial detection + bounding box, then classical CV on cropped ROI for sub-pixel centroid precision.

### 3.5 Computing Center and Real-World Offset

**Step 1 -- Centroid via Image Moments:**
```python
M = cv2.moments(contour)
cX = M["m10"] / M["m00"]
cY = M["m01"] / M["m00"]
```

**Step 2 -- Pixel to meters using GSD:**
```
GSD = (flight_height * sensor_width) / (focal_length * image_width)  # meters/pixel
dx_meters = (cX - image_width/2) * GSD
dy_meters = (cY - image_height/2) * GSD
```

**Step 3 -- Compensate for drone attitude:**
```
offset_NED = rotation_matrix(roll, pitch, yaw) * [dx_meters, dy_meters, 0]
```

### 3.6 Precision Centering (Visual Servoing)

**Image-Based Visual Servoing (IBVS) with PID:**
1. Detect target rectangle centroid in camera frame
2. Compute pixel error from image center
3. Convert to velocity command via PID controller
4. Send via `/ap_N/cmd_vel` (TwistStamped) in GUIDED mode

**ArduPilot integration:**
- Set drone to GUIDED mode
- Send velocity commands via `SET_POSITION_TARGET_LOCAL_NED` or `/ap_N/cmd_vel`
- Commands must be re-sent every second (vehicle stops after 3s with no command)
- Use `BODY_NED` frame for velocity relative to drone heading

**Achievable accuracy:** +/- 10-20 cm centering with well-calibrated system.

### 3.7 Size at Various Altitudes

| Zone | Real Size | Pixels at 30ft | Pixels at 15ft |
|------|-----------|----------------|----------------|
| WA | 20x20 ft (6.1m) | ~953 px | ~1906 px |
| L/H | 15x15 ft (4.57m) | ~714 px | ~1429 px |
| F1 | 7x7 ft (2.13m) | ~333 px | ~667 px |
| F2 | 3x3 ft (0.91m) | ~143 px | ~286 px |

All zones are clearly detectable at both altitudes with a 1920x1440 camera.

### Sources
- [OpenCV Canny Edge Detection](https://docs.opencv.org/4.x/da/d22/tutorial_py_canny.html)
- [OpenCV Hough Line Transform](https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html)
- [IBVS Drone Interception -- ArXiv](https://arxiv.org/html/2409.17497v1)
- [Ultralytics NVIDIA Jetson Guide](https://docs.ultralytics.com/guides/nvidia-jetson/)
- [YOLOv8 TensorRT Jetson Deployment](https://wiki.seeedstudio.com/YOLOv8-TRT-Jetson/)

---

## 4. Autonomous Mission Scripting

### 4.1 Framework Comparison

| Framework | Recommendation | Notes |
|-----------|---------------|-------|
| **pymavlink** | PRIMARY | Actively maintained, full MAVLink access, reliable with ArduPilot 4.5.7 |
| **Lua Scripting** | COMPLEMENT | Runs on flight controller, excellent for servo operations |
| **DroneKit-Python** | AVOID | Abandoned for 5+ years, compatibility issues |
| **MAVSDK-Python** | AVOID | Optimized for PX4, `goto_location` returns "Command Denied" with ArduCopter |
| **MAVROS2** | PARTIAL USE | Good for telemetry, unreliable for LANDING_TARGET |

**Best approach:** pymavlink on Jetson Orin as primary mission controller + Lua scripts on flight controller for time-critical servo sequencing.

### 4.2 GUIDED Mode Commands

**SET_POSITION_TARGET_GLOBAL_INT:**
```python
master.mav.set_position_target_global_int_send(
    int(1e3 * (time.time() - boot_time)),
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    0x0DF8,  # position only type_mask
    int(lat * 1e7), int(lon * 1e7), alt,
    0, 0, 0,  # velocity
    0, 0, 0,  # acceleration
    0, 0)     # yaw, yaw_rate
```

**Mode switching:**
```python
mode_id = master.mode_mapping()['GUIDED']  # Returns 4
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)
```

**Key:** With `MIS_RESTART=0`, AUTO mode resumes from last waypoint after GUIDED detour. Use `MAV_CMD_DO_SET_MISSION_CURRENT` to jump to specific waypoints for FM-3 loops.

### 4.3 Mission Commands for DBVF

#### Navigation Commands

| Command | Usage |
|---------|-------|
| NAV_TAKEOFF | Vertical climb to specified altitude |
| NAV_WAYPOINT | Fly straight to lat/lon/alt for transit legs |
| NAV_LAND | Descend vertically (precision landing engages if PLND_ENABLED) |
| NAV_LOITER_TIME | Hover at location for stabilization before drops |
| NAV_RETURN_TO_LAUNCH | Return to Home as final command |
| NAV_PAYLOAD_PLACE | Auto-descend, detect ground contact, release gripper, ascend |

#### DO Commands

| Command | Usage |
|---------|-------|
| DO_SET_SERVO | Set servo PWM for payload release |
| DO_GRIPPER | Open (0) or close (1) gripper |
| DO_CHANGE_SPEED | Adjust speed mid-mission |
| DO_JUMP | Loop back to mission item for FM-3 cycles (-1 = infinite) |
| DO_SET_MODE | Switch flight mode from within mission |
| NAV_SCRIPT_TIME | Execute Lua script during AUTO mode |

### 4.4 Payload Release Mechanism Control

**DO_SET_SERVO via MAVLink:**
```python
def set_servo_pwm(master, servo_n, pwm):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
        servo_n, pwm, 0, 0, 0, 0, 0)
```

**Important:** Set `SERVOx_FUNCTION = 0` (Disabled) for servos controlled via DO_SET_SERVO, otherwise values get overwritten.

**Sequential multi-payload drop options:**
1. **Multiple servos** (one per payload) -- most reliable
2. **Single servo with Lua sequencing** -- mechanical indexing mechanism
3. **DO_GRIPPER** with GRIP_TYPE=1 -- limited to binary open/close

### 4.5 State Machine Architecture

**Top-Level States:**
```
IDLE -> FM1_TAKEOFF -> FM1_TRANSIT -> FM1_PRECISION_LAND -> FM1_LANDED
  -> FM2_TAKEOFF -> FM2_TRANSIT -> FM2_DROP_RED -> FM2_RETURN
  -> FM3_TRANSIT_WA -> FM3_LAND_WA -> FM3_PICKUP -> FM3_TAKEOFF
  -> FM3_TRANSIT_DROP -> FM3_DROP_YELLOW -> FM3_LOOP_CHECK
  -> RTH -> LANDED -> COMPLETE
```

**Hybrid AUTO/GUIDED pattern:**
1. Upload mission waypoints for transit legs
2. Switch to AUTO for navigation
3. Switch to GUIDED for precision operations (centering, descent)
4. Switch to LAND for precision landing with PLND
5. Resume AUTO for next transit leg

**Safety requirements per state:**
- Every state has manual override path
- Watchdog timers per state
- Altitude verification before horizontal transit
- Battery monitoring throughout
- Position tolerance checks before proceeding

### 4.6 Lua Scripting (ArduPilot 4.5.7)

**Capabilities:**
- `servo:set_output_pwm(channel, pwm)` -- servo control
- `vehicle:set_mode(mode_number)` / `vehicle:get_mode()`
- `ahrs:get_location()`, `ahrs:get_relative_home_altitude()`
- `gps:location(0)`, `gps:ground_speed(0)`
- `battery:voltage(0)`, `battery:current_amps(0)`
- RC input: channels via `RCx_OPTION` 300-307
- `gcs:send_text(severity, message)` for status reporting
- NAV_SCRIPT_TIME for mission-integrated custom behaviors

**Setup:** `SCR_ENABLE = 1`, place `.lua` files in `APM/scripts/` on SD card.

**Companion computer coordination:**
- RC channel overrides: Companion sets RC value, Lua reads as trigger
- Parameters: Companion sets `SCR_USER1-6`, Lua reads them
- Named float channels for inter-script communication

### 4.7 ROS2 + MAVLink Integration

**ArduPilot Native DDS (already in workspace):**

Published topics (per drone `/ap_N/`):
- `/ap/pose/filtered` -- EKF position
- `/ap/geopose/filtered` -- Global coordinates
- `/ap/navsat` -- GPS data
- `/ap/twist/filtered` -- Velocity
- `/ap/battery` -- Battery state

Subscribed topics:
- `/ap/cmd_vel` -- Velocity commands
- `/ap/cmd_gps_pose` -- GPS waypoint goals
- `/ap/joy` -- RC overrides

Services:
- `/ap/arm_motors` -- Arm/disarm
- `/ap/mode_switch` -- Mode changes

**Critical:** DDS does not expose LANDING_TARGET subscription. Precision landing messages MUST go through MAVLink (pymavlink over serial UART).

### 4.8 Geofence and Failsafe

**Required competition parameters:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| FS_GCS_ENABLE | 5 | Land on GCS loss |
| FS_GCS_TIMEOUT | 5 seconds | GCS heartbeat timeout |
| RC_FS_TIMEOUT | 5 seconds | RC failsafe timeout |
| FS_THR_ENABLE | 3 | Land on throttle failsafe |
| FS_OPTIONS | Bit 1 | Continue in Auto during GCS failsafe |

**Geofence:** Set FENCE_ENABLE=1 with polygon inclusion fence around competition field, FENCE_ALT_MAX ~50m.

**Companion computer must send heartbeats at >= 1 Hz** or GCS failsafe triggers.

### 4.9 Altitude Management

- All transit at >=30ft AGL. Set transit waypoints to 10-12m relative altitude for safety margin.
- Competition field is flat (SURVICE ATO), so relative altitude ≈ AGL.
- Enable `WPNAV_RFND_USE = 1` for rangefinder-based AGL during low-altitude operations.
- For drops at >=15ft: descend in GUIDED mode to 16-18ft AGL using rangefinder.

### Sources
- [Copter Commands in Guided Mode](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)
- [Lua Scripts -- Copter documentation](https://ardupilot.org/copter/docs/common-lua-scripts.html)
- [Copter Mission Command List](https://ardupilot.org/copter/docs/mission-command-list.html)
- [Servo Gripper](https://ardupilot.org/copter/docs/common-gripper-servo.html)
- [GCS Failsafe](https://ardupilot.org/copter/docs/gcs-failsafe.html)
- [Fences -- Copter](https://ardupilot.org/copter/docs/common-geofencing-landing-page.html)
- [ROS 2 Interfaces](https://ardupilot.org/dev/docs/ros2-interfaces.html)
- [pymavlink vs MAVSDK-Python -- ArduPilot Forum](https://discuss.ardupilot.org/t/pymavlink-vs-mavsdk-python-vs-dronekit-python-for-udp-receiving-program/86422)

---

## 5. Payload Drop Precision Optimization

### 5.1 Ballistic Drop Model

**Terminal velocity of 0.5 lb sand bag:** ~17-21 m/s (NOT reached from 15-30ft drops)

**Drop time and impact velocity (vacuum approximation, air resistance negligible at these heights):**

| Height | Meters | Fall Time | Impact Velocity |
|--------|--------|-----------|----------------|
| 15 ft | 4.57 m | **0.97 s** | **9.47 m/s (21.2 mph)** |
| 20 ft | 6.10 m | **1.12 s** | **10.94 m/s (24.5 mph)** |
| 25 ft | 7.62 m | **1.25 s** | **12.23 m/s (27.4 mph)** |
| 30 ft | 9.14 m | **1.37 s** | **13.39 m/s (30.0 mph)** |

### 5.2 Wind Drift Analysis

**Realistic drift model** (bag acquires ~50% of wind speed during short fall):

| Wind Speed | 15 ft drop | 20 ft drop | 30 ft drop |
|------------|-----------|-----------|-----------|
| 5 mph (2.2 m/s) | ~1.1 m (3.5 ft) | ~1.2 m (4.1 ft) | ~1.5 m (5.0 ft) |
| 10 mph (4.5 m/s) | ~2.1 m (7.1 ft) | ~2.5 m (8.2 ft) | ~3.1 m (10.1 ft) |
| 15 mph (6.7 m/s) | ~3.2 m (10.7 ft) | ~3.7 m (12.3 ft) | ~4.6 m (15.0 ft) |

**Churchville, MD April weather:** Average wind 14.8 mph, gusts 21-30 mph expected. The 2024-2025 competition at the same venue had 20+ mph winds and rain.

### 5.3 Wind Compensation

**ArduPilot EKF wind estimation:**
- Enable: `EK3_DRAG_BCOEF_X`, `EK3_DRAG_BCOEF_Y` = mass/frontal_area
- Access: WIND MAVLink message (`wind_dir`, `wind_vel`)
- Limitations: Estimates degrade while hovering stationary

**Release point offset:**
```
release_offset = -wind_vector * fall_time * coupling_factor
# coupling_factor ~0.4-0.6 for compact bags from 15ft
```

**When precision becomes impractical:**
- F2 (3x3 ft = 0.91m): Unreliable above ~8-10 mph wind
- F1 (7x7 ft = 2.13m): Unreliable above ~15-18 mph wind

### 5.4 Optimal Drop Altitude

**Recommendation: 16-18 ft AGL**
- Minimal extra drift vs 15ft minimum (~5-10% more)
- Small safety buffer against altitude measurement error
- Minimum possible fall time for wind drift reduction
- Use rangefinder for precise AGL measurement

### 5.5 Release Mechanism Design

**Recommended:** Servo-actuated pin-release mechanism
- Individual servo per payload slot (5x SG90, ~45g total) -- most reliable
- Or single servo with mechanical indexing mechanism (lighter but more complex)
- Mount at or below drone CG
- Pin-pull or door-drop design (bag falls under gravity, zero lateral impulse)
- 3D-printable (ABS or PLA+)
- **Drop individually** -- allows position correction between drops, less stability disruption

### 5.6 Drone Stability at Drop Point

- GPS loiter accuracy: +/- 0.45m (even with RTK)
- Precision Loiter with CV: +/- 5-15 cm (well-tuned)
- **Wait 3-5 seconds** after arriving at drop point before releasing
- Release only when velocity < 0.1 m/s and position error < threshold
- 0.5 lb (227g) release = 4.5-7.5% weight change on 3-5 kg drone -- manageable

### 5.7 Impact Survivability (Anti-Shatter)

From 15ft: impact at 9.47 m/s (21.2 mph), KE = 10.2 joules.

**Recommendations:**
- **Material:** Heavy-duty woven polypropylene or ripstop nylon
- **Double-bag:** Inner bag inside outer bag
- **Seams:** Double-stitched or heat-sealed, especially corners
- **Fill:** 70-80% capacity (allows deformation on impact)
- **Closure:** Fold-and-stitch or heat-seal (not just tied)
- **Testing:** Drop test exact bags 20+ times from 20ft onto concrete before competition

### 5.8 Scoring Strategy

**Expected value analysis (per bag):**
- F1 at 90% hit rate: 2.5 * 0.9 = 2.25 pts/bag
- F2 at 67% hit rate: 5.0 * 0.67 = 3.35 pts/bag
- F2 has higher expected value BUT catastrophic shatter risk

**Recommended strategy:**
1. **Calm (< 5 mph):** All bags at F2 -- highest expected value
2. **Moderate (5-12 mph):** Split -- 2-3 bags at F2, rest at F1
3. **Strong (> 12 mph):** All bags at F1 -- F2 becomes unreliable
4. **Never rush** -- a shatter zeros ALL payload points across ALL missions

**Autonomy multipliers make huge difference:**
- FM-2 autonomous: x2 multiplier on drops
- FM-3 from WA autonomous: x3 multiplier + 50 bonus points
- 5 bags at F2 with x3: 5 * 5 * 3 = **75 pts per cycle**

### 5.9 FM-3 Multiple Cycle Optimization

**Estimated cycle time:**
1. Land at WA (precision landing): 15-25s
2. Ground time (pickup): 30-60s
3. Takeoff to 30ft: 5-10s
4. Transit to F1/F2 (~100m at 5 m/s): 20-40s
5. Descend + stabilize + drop 5 bags: 15-30s
6. Climb + transit back: 25-50s

**Total: ~2-3.5 minutes per cycle**

Plan for **2-3 complete cycles** in the 10-minute window. Minimize ground time at WA with pre-loaded payload bundles.

### Sources
- [Autonomous Ballistic Airdrop -- Springer](https://link.springer.com/article/10.1007/s10514-020-09902-3)
- [ArduPilot Wind Estimation](https://ardupilot.org/copter/docs/airspeed-estimation.html)
- [ArduPilot Servo Gripper](https://ardupilot.org/copter/docs/common-gripper-servo.html)
- [Churchville MD April Weather](https://wanderlog.com/weather/62912/4/churchville-weather-in-april)
- [Algorithm for Precise Payload Drop from FPV Drone](https://www.researchgate.net/publication/385542994)

---

## 6. Recommended System Architecture

### 6.1 Hardware

| Component | Recommended |
|-----------|-------------|
| Compute | Jetson Orin Nano Super 8GB (15W, 67 TOPS, ~8g) |
| Carrier Board | Seeed reComputer Mini (80g, XT30) or Auvidea JNX110 |
| Camera | IMX477 CSI with wide-angle lens (114-160 deg HFOV) |
| Rangefinder | Downward-facing LiDAR (TFmini-S or similar) |
| Flight Controller | ArduPilot Copter 4.5.7 |

### 6.2 Software Stack

```
Jetson Orin (JetPack 6.1, ROS2 Humble, Isaac ROS 3.2)
├── isaac_ros_argus_camera (CSI, zero-copy GPU)
├── isaac_ros_image_proc/rectify (GPU undistort)
├── isaac_ros_apriltag (CUDA, tag36h11, 120+ fps)
├── drop_zone_detector (CPU OpenCV or GPU YOLO11n)
├── precision_landing_node (tag pose -> LANDING_TARGET @ 20Hz)
├── mission_state_machine (pymavlink, hierarchical FSM)
└── safety_monitor (altitude/battery/geofence checks)

ArduPilot Copter 4.5.7
├── DDS Bridge (telemetry to ROS2)
├── MAVLink (pymavlink for commands + LANDING_TARGET)
├── Lua: payload_drop_sequencer.lua
├── PLND: precision landing at WA
└── AUTO/GUIDED/LAND mode switching
```

### 6.3 Data Flow

```
[IMX477 CSI Camera]
    |
    v
[isaac_ros_argus_camera] -- GPU memory (NVMM), zero-copy
    |
    v
[isaac_ros_image_proc/rectify] -- GPU, undistort wide-angle
    |
    +---> [isaac_ros_apriltag] -- GPU/CUDA, 120+ fps
    |         |
    |         v
    |     [precision_landing_node] -- CPU
    |         | Dual-tag switching logic
    |         | Pose -> angle_x/angle_y or x/y/z
    |         v
    |     [pymavlink] -- LANDING_TARGET @ 20Hz -> ArduPilot UART
    |
    +---> [drop_zone_detector] -- CPU (OpenCV) or GPU (YOLO)
              |
              v
          [visual_servoing_node] -- CPU
              | PID controller, wind compensation
              | Centering velocity commands
              v
          [/ap_N/cmd_vel] -- ROS2 DDS -> ArduPilot

[mission_state_machine] -- pymavlink
    | Mode switches, waypoint commands, servo triggers
    | Heartbeats @ 1Hz, telemetry monitoring
    v
[ArduPilot] -- AUTO/GUIDED/LAND modes
    |
    v
[Lua: payload_drop_sequencer]
    | Triggered by RC channel override from companion
    | Sequential servo actuation for individual bag drops
```

### 6.4 Mission Sequence (Full Attempt)

```
Phase 1: FM-1 (Takeoff + Land at L)
  1. ARM (GUIDED) -> Takeoff to 30ft
  2. Switch to AUTO -> Transit H to L at >=30ft AGL
  3. Arrive near L -> Switch to GUIDED
  4. Descend -> Switch to LAND mode (no PLND at L, GPS only)
  5. Land within L (15x15ft) -> FM-1 complete (+20 pts, +30 if autonomous)

Phase 2: FM-2 (Payload Drop)
  6. Takeoff from L to 30ft (GUIDED)
  7. Switch to AUTO -> Transit to F1 or F2
  8. Arrive over drop zone -> Switch to GUIDED
  9. Descend to 16-18ft AGL using rangefinder
  10. Visual servoing to center over wind-compensated release point
  11. Stabilize (vel < 0.1 m/s, 3-5s) -> Trigger servo drops (individual)
  12. Climb to 30ft -> FM-2 complete

Phase 3: FM-3 (Pickup + Drop, loop)
  13. Transit to WA (AUTO at 30ft)
  14. Arrive over WA -> Switch to GUIDED
  15. Descend -> LAND mode with PLND (AprilTag precision landing)
  16. Land at WA center -> Pickup yellow payloads
  17. ARM -> Takeoff to 30ft (GUIDED)
  18. Transit to F1/F2 (AUTO)
  19. Descend + center + drop (GUIDED, same as FM-2)
  20. Repeat 13-19 for additional cycles if time permits

Phase 4: RTH
  21. Transit back to Home (H)
  22. Land vertically at Home before 10-minute timer expires
```
