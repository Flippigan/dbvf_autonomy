# Jetson Orin Nano + Cube Orange Setup Guide

Complete instructions for wiring, configuring, and deploying the DBVF autonomy stack on physical hardware.

---

## Hardware Required

| Component | Model | Notes |
|-----------|-------|-------|
| Flight controller | Cube Orange | Standard (no Ethernet variant) |
| Companion computer | Jetson Orin Nano 4GB | JetPack 6.x (L4T R36.x, Ubuntu 22.04) |
| Camera | Arducam IMX219 8MP | CSI connector, 120-degree FOV recommended |
| Serial cable | JST-GH 6-pin to Dupont | 3 wires: TX, RX, GND |

---

## Part 1: Physical Wiring

### 1.1 Cube Orange TELEM2 → Orin Nano UART1

Connect three wires between Cube Orange TELEM2 (6-pin JST-GH) and Orin Nano J12 header:

| Cube Orange TELEM2 Pin | Signal   | Wire Color (suggested) | Orin Nano J12 Pin | Signal    |
|------------------------|----------|------------------------|-------------------|-----------|
| Pin 2                  | TX (out) | Green                  | Pin 10            | UART1 RX  |
| Pin 3                  | RX (in)  | Yellow                 | Pin 8             | UART1 TX  |
| Pin 6                  | GND      | Black                  | Pin 6 (any GND)   | GND       |

**Important:**
- Do **NOT** connect Pin 1 (5V). The Orin has its own power supply.
- Both sides are 3.3V logic — no level shifter is needed.
- TX on Cube goes to RX on Orin (crossover).

### 1.2 CSI Camera

1. Power off the Jetson Orin Nano.
2. Locate the CSI camera connector on the carrier board (typically labeled CAM0).
3. Lift the connector latch.
4. Insert the Arducam IMX219 ribbon cable with contacts facing the board.
5. Press the latch down to lock.
6. Mount the camera pointing straight down on the drone frame (hardmount, no gimbal).

---

## Part 2: Cube Orange Configuration

Set these ArduPilot parameters on the Cube Orange using QGroundControl or MAVProxy. These configure TELEM2 for MAVLink communication with the Orin.

### 2.1 Serial Port Parameters

```
SERIAL2_PROTOCOL = 2      # MAVLink2 on TELEM2
SERIAL2_BAUD = 921        # 921600 baud
```

### 2.2 Precision Landing Parameters

Copy all PLND parameters from `src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm`. The precision landing parameters are identical between sim and real — MAVLink LANDING_TARGET messages are transport-agnostic.

Key parameters to verify:

```
PLND_ENABLED = 1
PLND_TYPE = 1             # MAVLink
PLND_EST_TYPE = 1         # Kalman filter
PLND_ACC_P_NSE = 0.5
```

### 2.3 System ID

```
MAV_SYS_ID = 1            # Vehicle system ID (default)
```

---

## Part 3: Jetson Orin Nano Software Setup

### 3.1 Flash JetPack (if not already done)

Use NVIDIA SDK Manager on a host PC to flash JetPack 6.x. This provides:
- Ubuntu 22.04
- CUDA, cuDNN, TensorRT (unused but included)
- GStreamer with `nvarguscamerasrc` (required for CSI camera)

### 3.2 Configure UART

The Orin Nano's UART1 (`/dev/ttyTHS1`) is used as a serial console by default. Disable it:

```bash
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty

# Add your user to the dialout group for serial access
sudo usermod -aG dialout $USER

# Reboot for group change to take effect
sudo reboot
```

After reboot, verify:
```bash
ls -la /dev/ttyTHS1
# Should show: crw-rw---- 1 root dialout ...
```

### 3.3 Install ROS2 Humble

Use `ros-base` (not `ros-desktop`) to save ~1.5GB — no rviz or Gazebo needed on the aircraft.

```bash
# Add ROS2 apt repository (if not already added)
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble base + required packages
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-image-transport \
  ros-humble-camera-info-manager \
  ros-humble-cv-bridge \
  python3-colcon-common-extensions \
  python3-pip
```

Add to `~/.bashrc`:
```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 3.4 Install pymavlink

```bash
pip3 install pymavlink==2.4.43
```

### 3.5 Build apriltag C Library

```bash
cd /tmp
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
cmake -B build
cmake --build build -j$(nproc)
sudo cmake --install build
sudo ldconfig
```

### 3.6 Set Up the Workspace

Copy or clone the workspace packages onto the Jetson. Only these are needed:

```
src/
  dbvf_msgs/          # Messages and services
  dbvf_autonomy/      # Autonomy nodes (6 nodes)
  apriltag_ros/       # v3.3.0
  apriltag_msgs/      # Message definitions
```

Do **not** copy `ardupilot_gazebo`, `ardupilot_gz_bringup`, or `formation_*` — these are sim-only.

### 3.7 Build the Workspace

```bash
cd ~/ardu_ws   # or wherever you placed the workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select dbvf_msgs apriltag_msgs apriltag_ros dbvf_autonomy
source install/setup.bash
```

---

## Part 4: Camera Calibration

The real IMX219 needs a calibration file for accurate AprilTag pose estimation. The simulation uses hardcoded intrinsics from Gazebo — the real camera will differ.

### 4.1 Install Calibration Tool

```bash
sudo apt install -y ros-humble-camera-calibration
```

### 4.2 Print a Calibration Target

Print an 8x6 checkerboard with 25mm squares. Measure the actual printed square size (printers may scale).

### 4.3 Run Calibration

Terminal 1 — start the camera:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run dbvf_autonomy csi_camera_node --ros-args \
  -p sensor_id:=0 -p width:=1280 -p height:=720 -p framerate:=30
```

Terminal 2 — run calibrator:
```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args -r image:=/camera/image -r camera_info:=/camera/camera_info
```

Move the checkerboard around until all bars are green. Click "Calibrate", then "Save".

### 4.4 Install Calibration File

```bash
# The calibrator saves to /tmp/calibrationdata.tar.gz
tar xzf /tmp/calibrationdata.tar.gz -C /tmp/calibration
cp /tmp/calibration/ost.yaml ~/ardu_ws/src/dbvf_autonomy/config/imx219_calibration.yaml
```

### 4.5 Update Config

Edit `src/dbvf_autonomy/config/hardware_params.yaml` and set the correct path:

```yaml
camera:
  ros__parameters:
    camera_info_url: "file:///home/<user>/ardu_ws/src/dbvf_autonomy/config/imx219_calibration.yaml"
```

---

## Part 5: Verification Steps

Run these tests in order before flying.

### 5.1 Verify Serial Link

Test that the Orin can communicate with the Cube Orange over MAVLink:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

# Run just the mavlink interface node
ros2 run dbvf_autonomy mavlink_interface_node --ros-args \
  -p connection_type:=serial \
  -p serial_device:=/dev/ttyTHS1 \
  -p serial_baud:=921600

# In another terminal, check for heartbeat
ros2 topic echo /dbvf/heartbeat_status
# Should show: data: true

# Check vehicle state
ros2 topic echo /dbvf/vehicle_state
# Should show mode, armed status, GPS coordinates
```

### 5.2 Verify Camera Pipeline

Test that the CSI camera publishes images:

```bash
ros2 run dbvf_autonomy csi_camera_node --ros-args \
  -p sensor_id:=0 -p width:=1280 -p height:=720 -p framerate:=30

# In another terminal
ros2 topic hz /camera/image
# Should show ~30 Hz
```

### 5.3 Verify AprilTag Detection

Hold an AprilTag (tag36h11, ID 1 or 2) in front of the camera:

```bash
# Start camera + apriltag_ros + tag adapter
ros2 launch dbvf_autonomy precision_landing_real.launch.py

# In another terminal
ros2 topic echo /dbvf/tag_status
# Should show detected: true, tag_id: 1 (or 2)

ros2 topic echo /dbvf/landing_target_pose
# Should show angle_x, angle_y values changing as you move the tag
```

### 5.4 Full Stack Test (on the ground, propellers OFF)

```bash
ros2 launch dbvf_autonomy mission_real.launch.py

# Verify all topics are publishing
ros2 topic list | grep dbvf

# Expected topics:
# /dbvf/vehicle_state
# /dbvf/heartbeat_status
# /dbvf/landing_target_pose
# /dbvf/tag_status
# /dbvf/landing_state
# /dbvf/mission_state
# /dbvf/mission_phase
```

---

## Part 6: Running a Mission

### 6.1 Pre-Flight Checklist

1. Propellers installed, battery connected
2. Cube Orange powered and GPS lock acquired
3. Jetson Orin Nano powered (separate BEC or USB-C power bank)
4. Serial cable connected (TX/RX/GND)
5. CSI camera connected and facing down
6. `hardware_params.yaml` has correct `camera_info_url`
7. `mission_params.yaml` has correct GPS waypoints for the competition field

### 6.2 Launch

On the Jetson:
```bash
cd ~/ardu_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch dbvf_autonomy mission_real.launch.py
```

### 6.3 Start Mission

Via RC transmitter (if configured with ch14/ch15 switches):
- Flip ch14 high to start mission

Or via command line:
```bash
ros2 service call /dbvf/start_mission dbvf_msgs/srv/StartMission "{}"
```

### 6.4 Monitor

```bash
ros2 topic echo /dbvf/mission_state    # Current FSM state
ros2 topic echo /dbvf/mission_phase    # FM1, FM2, FM3, RTH
ros2 topic echo /dbvf/vehicle_state    # GPS, altitude, mode
```

### 6.5 Resume After Flagger

After the drone lands at L and waits for the flagger:
- Flip ch15 high on RC transmitter, OR:
```bash
ros2 service call /dbvf/resume_mission dbvf_msgs/srv/ResumeMission "{}"
```

Resume automatically sets GUIDED mode, arms, and takes off — no manual steps needed.

### 6.6 Abort

```bash
ros2 service call /dbvf/abort_mission dbvf_msgs/srv/AbortMission "{}"
```

---

## Part 7: Troubleshooting

### No heartbeat from Cube Orange

1. Check wiring: TX/RX are crossed (Cube TX → Orin RX, Cube RX → Orin TX)
2. Check baud rate: `SERIAL2_BAUD=921` on Cube, `serial_baud: 921600` in config
3. Check `SERIAL2_PROTOCOL=2` on Cube
4. Verify `/dev/ttyTHS1` exists: `ls -la /dev/ttyTHS1`
5. Verify user is in dialout group: `groups` should show `dialout`

### CSI camera not opening

1. Verify camera ribbon cable is fully seated
2. Test with: `nvgstcapture-1.0` (NVIDIA's test app)
3. Check `sensor_id` — try 0 or 1 depending on which CSI port
4. Verify JetPack includes `nvarguscamerasrc`: `gst-inspect-1.0 nvarguscamerasrc`

### AprilTags not detected

1. Verify camera calibration file exists and path is correct in `hardware_params.yaml`
2. Check tag family is `tag36h11` and IDs are 1 and 2
3. Ensure adequate lighting — the IMX219 needs decent light for tag detection
4. Check tag sizes match config: primary = 0.15m, secondary = 0.05m (measure the printed tags)

### Connection type errors

The `connection_type` parameter must be exactly `"serial"`, `"tcp"`, or `"udp"`. If using `hardware_params.yaml`, this is set to `"serial"` by default. If you need to connect via UDP (e.g., through `mavlink-routerd`), change to `"udp"` and set `udp_host`/`udp_port`.

---

## Appendix: Memory Budget

| Component                          | Estimated RAM |
|------------------------------------|---------------|
| JetPack 6 base                     | ~600MB        |
| ROS2 Humble (ros-base)             | ~150MB        |
| 6 autonomy nodes + pymavlink       | ~100MB        |
| CSI camera pipeline (nvarguscamerasrc) | ~200MB    |
| apriltag_ros                       | ~100MB        |
| **Total**                          | **~1.15GB**   |
| **Remaining headroom (of 4GB)**    | **~2.85GB**   |

## Appendix: Future — Adding QGC Access

When you need QGC telemetry alongside autonomous operation:

1. Install `mavlink-routerd` on the Orin:
   ```bash
   sudo apt install mavlink-router
   ```

2. Configure it to own `/dev/ttyTHS1` and expose UDP endpoints:
   ```ini
   # /etc/mavlink-router/main.conf
   [General]
   TcpServerPort = 0

   [UartEndpoint serial]
   Device = /dev/ttyTHS1
   Baud = 921600

   [UdpEndpoint local]
   Mode = Normal
   Address = 127.0.0.1
   Port = 14550

   [UdpEndpoint gcs]
   Mode = Normal
   Address = 0.0.0.0
   Port = 14551
   ```

3. Update `hardware_params.yaml`:
   ```yaml
   mavlink:
     connection_type: "udp"
     udp_host: "127.0.0.1"
     udp_port: 14550
   ```

4. QGC on GCS laptop connects to `<orin-ip>:14551`.

No code changes required.
