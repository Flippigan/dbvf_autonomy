# SITL to Physical Aircraft Transition Guide

Guide for deploying the `dbvf_autonomy` ROS2 stack from Gazebo SITL on a development machine to a **Jetson Orin** companion computer connected to a **Cube Orange** flight controller.

---

## 1. Hardware Wiring: Cube Orange TELEM2 to Jetson Orin

Use **TELEM2** (SERIAL2) on the Cube Orange — the standard companion computer port.

### Option A: Direct UART (recommended — lowest latency)

Wire the Cube Orange TELEM2 connector to the Jetson Orin 40-pin header:

| Cube Orange TELEM2 | Jetson Orin 40-pin Header         |
|---------------------|-----------------------------------|
| TX (pin 2)          | UART RX (pin 10, `/dev/ttyTHS1`)  |
| RX (pin 3)          | UART TX (pin 8)                   |
| GND (pin 6)         | GND (pin 6)                       |

Both are 3.3V logic — no level shifter needed. Do **not** connect the VCC/5V pin.

### Option B: USB-to-Serial Adapter

Plug a USB-to-serial adapter (FTDI or similar) into the Jetson Orin's USB port with a TELEM-to-USB cable. The device appears as `/dev/ttyUSB0` or `/dev/ttyACM0`. Simpler wiring but adds latency.

---

## 2. ArduPilot Parameters (Cube Orange)

Set via Mission Planner or MAVProxy on the Cube Orange:

### TELEM2 Serial Configuration

```
SERIAL2_PROTOCOL = 2        # MAVLink2
SERIAL2_BAUD     = 921      # 921600 baud (ArduPilot uses kbaud notation)
```

### Precision Landing (PLND)

Ensure these are loaded (already defined in `src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm` for sim — same params apply to hardware):

```
PLND_ENABLED   = 1
PLND_TYPE      = 1          # MAVLink
PLND_EST_TYPE  = 1          # Kalman filter
```

### Rangefinder (if equipped)

```
RNGFND1_TYPE   = <your sensor type>
RNGFND1_MIN_CM = <min range cm>
RNGFND1_MAX_CM = <max range cm>
```

The precision landing node's slow descent feature (`slow_descent_altitude`, `slow_descent_rate`) uses rangefinder data from `/dbvf/vehicle_state.range_alt`. Without a rangefinder, it falls back gracefully (range_alt = -1.0 sentinel value, descent rate passthrough unchanged).

---

## 3. Jetson Orin Directory Structure

Mirror the development machine layout:

```
~/ardu_ws/                        # Colcon workspace root
├── src/
│   ├── dbvf_autonomy/            # Copy from dev machine
│   ├── dbvf_msgs/                # Copy from dev machine
│   ├── apriltag_ros/             # Clone v3.3.0
│   └── apriltag_msgs/            # Clone v2.0.1
├── install/                      # Built natively on Jetson (ARM64)
├── build/
└── log/
```

**Important:** You cannot copy `install/` or `build/` from an x86 dev machine — they must be built natively on the Jetson (ARM64). Copy only `src/`.

### Initial Setup

```bash
# Transfer source to Jetson
mkdir -p ~/ardu_ws/src
rsync -av devmachine:~/Documents/ardu_ws/src/ ~/ardu_ws/src/

# Install apriltag C library (build from source for ARM64)
cd /tmp && git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag && cmake -B build -DCMAKE_INSTALL_PREFIX=~/.local
cmake --build build -j$(nproc) && cmake --install build

# Install pymavlink
pip3 install pymavlink

# Build workspace
cd ~/ardu_ws
source /opt/ros/humble/setup.bash
CMAKE_PREFIX_PATH="$HOME/.local:$CMAKE_PREFIX_PATH" \
  colcon build --packages-select dbvf_msgs apriltag_msgs apriltag_ros dbvf_autonomy
```

### Serial Port Permissions

The Jetson UART device requires the `dialout` group:

```bash
sudo usermod -aG dialout $USER
# Log out and back in for the group change to take effect

# Quick test (resets on reboot):
sudo chmod 666 /dev/ttyTHS1
```

---

## 4. Code Change: Serial Baud Rate Parameter

The `mavlink_interface_node` currently passes only `connection_string` to `mavutil.mavlink_connection()`. For serial connections, pymavlink also needs a `baud` parameter. Add this to the node:

In `mavlink_interface_node.py`, add the parameter declaration in `__init__`:

```python
self.declare_parameter('baud_rate', 921600)
```

And pass it in `_connect()`:

```python
def _connect(self):
    conn_str = self.get_parameter('connection_string').value
    src_sys = self.get_parameter('source_system').value
    src_comp = self.get_parameter('source_component').value
    baud = self.get_parameter('baud_rate').value
    try:
        self.conn = mavutil.mavlink_connection(
            conn_str, source_system=src_sys, source_component=src_comp,
            baud=baud)
        ...
```

For UDP/TCP connections (SITL), pymavlink ignores the `baud` kwarg, so this change is backwards-compatible with simulation.

---

## 5. Configuration: `jetson_params.yaml`

Update `src/dbvf_autonomy/config/jetson_params.yaml` — currently identical to `sim_params.yaml`. Required changes:

```yaml
mavlink_interface:
  ros__parameters:
    connection_string: "/dev/ttyTHS1"   # UART to Cube Orange TELEM2
    baud_rate: 921600                    # Must match SERIAL2_BAUD
    source_system: 255
    source_component: 0
    heartbeat_rate: 1.0
    vehicle_state_rate: 10.0
```

If using a USB adapter instead, change to:

```yaml
    connection_string: "/dev/ttyUSB0"   # USB-to-serial adapter
```

### Camera Body Transform

The `cam_body_x/y/z_from` parameters define how camera-frame axes map to body-frame axes. The current values were set for the Gazebo simulation camera mount:

```yaml
tag_detector_adapter:
  ros__parameters:
    cam_body_x_from: "-y"    # Body-X (forward) = camera -Y
    cam_body_y_from: "x"     # Body-Y (right)   = camera X
    cam_body_z_from: "z"     # Body-Z (down)     = camera Z
```

**You must verify these match your physical camera orientation.** See [Section 10: Bench Testing](#10-bench-testing-checklist-no-props) for how to validate.

### Mission Waypoints

Update `mission_params.yaml` with real competition GPS coordinates — the current values are Gazebo simulation coords (Canberra, Australia).

---

## 6. Camera Driver

In simulation, Gazebo publishes `/camera/image` and `/camera/camera_info` via the ros_gz bridge. On the Jetson you need a physical camera driver.

### USB Camera (V4L2)

```bash
sudo apt install ros-humble-usb-cam
```

### CSI Camera (IMX series via NVIDIA ISP)

```bash
# Option 1: gscam2
sudo apt install ros-humble-gscam

# Option 2: NVIDIA Isaac ROS (best performance on Jetson)
# See: https://nvidia-isaac-ros.github.io/
```

### Camera Calibration

The tag pose estimator requires accurate camera intrinsics (fx, fy, cx, cy from the K matrix in CameraInfo). Calibrate with a checkerboard:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args -r image:=/camera/image_raw -r camera_info:=/camera/camera_info
```

Save the resulting calibration YAML and configure your camera driver to publish it via the `camera_info_url` parameter.

---

## 7. Hardware Launch File

Create `src/dbvf_autonomy/launch/precision_landing_hw.launch.py`:

```python
"""Launch precision landing stack for physical hardware (Jetson Orin + Cube Orange)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('dbvf_autonomy')
    config = os.path.join(pkg_dir, 'config', 'jetson_params.yaml')

    return LaunchDescription([
        # Camera driver (adjust package/params for your camera)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera',
            parameters=[{
                'video_device': '/dev/video0',
                'camera_info_url': 'file:///home/finn/ardu_ws/camera_cal.yaml',
            }],
            remappings=[
                ('image_raw', '/camera/image'),
            ],
        ),

        # AprilTag detector
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            remappings=[
                ('image_rect', '/camera/image'),
                ('camera_info', '/camera/camera_info'),
                ('detections', '/apriltag/detections'),
            ],
            parameters=[{
                'family': '36h11',
                'size': 0.6,
                'tag.ids': [0, 1],
                'tag.sizes': [0.6, 0.15],
            }],
        ),

        # Tag detector adapter
        Node(
            package='dbvf_autonomy',
            executable='tag_detector_adapter_node',
            name='tag_detector_adapter',
            parameters=[config],
        ),

        # MAVLink interface (connects to Cube Orange via UART)
        Node(
            package='dbvf_autonomy',
            executable='mavlink_interface_node',
            name='mavlink_interface',
            parameters=[config],
        ),

        # Precision landing state machine
        Node(
            package='dbvf_autonomy',
            executable='precision_landing_node',
            name='precision_landing',
            parameters=[config],
        ),

        # Tag detection visualizer
        Node(
            package='dbvf_autonomy',
            executable='tag_visualizer_node',
            name='tag_visualizer',
        ),
    ])
```

Similarly create `mission_hw.launch.py` by copying `mission_sim.launch.py` and:
- Adding the camera driver node
- Replacing `sim_params.yaml` references with `jetson_params.yaml`
- Adding the mission_sequencer_node with `mission_params.yaml` (update GPS waypoints first)

---

## 8. How to Start on the Jetson

SSH in or plug in a monitor/keyboard. There is no ArduPilot SITL or Gazebo to launch — the Cube Orange **is** ArduPilot running on the FC hardware. Your stack just connects to it over serial.

### Start the Stack

```bash
# Terminal 1: Launch the autonomy stack
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
ros2 launch dbvf_autonomy precision_landing_hw.launch.py
```

### Monitor

```bash
# Terminal 2: Verify systems
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash

ros2 topic echo /dbvf/vehicle_state       # FC connection + telemetry
ros2 topic echo /dbvf/heartbeat_status     # MAVLink heartbeat alive
ros2 topic echo /dbvf/tag_status           # Camera + AprilTag detection
ros2 topic echo /dbvf/landing_state        # Precision landing FSM state
ros2 topic echo /dbvf/mission_state        # Mission sequencer state
ros2 topic list | grep dbvf               # All DBVF topics
```

### Trigger Precision Landing

```bash
ros2 service call /dbvf/start_precision_landing dbvf_msgs/srv/StartPrecisionLanding \
  "{target_lat: <REAL_LAT>, target_lon: <REAL_LON>}"
```

### Trigger Full Mission

```bash
ros2 service call /dbvf/start_mission dbvf_msgs/srv/StartMission "{}"
# After WAIT_FLAGGER state (drone landed at L), resume:
ros2 service call /dbvf/resume_mission dbvf_msgs/srv/ResumeMission "{}"
```

---

## 9. Optional: Auto-Start on Boot (systemd)

Create a systemd service to launch the stack automatically when the Jetson powers on:

```ini
# /etc/systemd/system/dbvf.service
[Unit]
Description=DBVF Autonomy Stack
After=network.target

[Service]
Type=simple
User=finn
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/finn/ardu_ws/install/setup.bash && ros2 launch dbvf_autonomy precision_landing_hw.launch.py'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start:

```bash
sudo systemctl enable dbvf
sudo systemctl start dbvf
# Check status:
sudo systemctl status dbvf
journalctl -u dbvf -f    # Live logs
```

---

## 10. Bench Testing Checklist (No Props!)

Before any flight, verify the full stack on the bench with props removed:

### MAVLink Connection

1. Launch just the mavlink_interface_node
2. Confirm `/dbvf/vehicle_state` publishes at 10Hz
3. Verify GPS coordinates, mode, and armed status are correct
4. Confirm `/dbvf/heartbeat_status` publishes `True`

### Camera and AprilTag Detection

1. Confirm `/camera/image` publishes frames
2. Hold a printed AprilTag (tag36h11, ID 0) in front of the camera
3. Verify `/apriltag/detections` shows the tag
4. Verify `/dbvf/tag_status` shows `detected: true` with correct `tag_id`

### Camera Body Transform Validation

1. Hold the drone over a tag (or hold a tag under the camera)
2. Move the tag to the drone's **left** — `angle_x` in `/dbvf/landing_target_pose` should be **negative**
3. Move the tag **forward** — `angle_y` should be **negative** (ahead of center)
4. If directions are wrong, adjust `cam_body_x/y/z_from` in `jetson_params.yaml`

### Servo Actuation

```bash
ros2 service call /dbvf/do_set_servo dbvf_msgs/srv/DoSetServo \
  "{servo_number: 9, pwm: 1100}"    # Release position
ros2 service call /dbvf/do_set_servo dbvf_msgs/srv/DoSetServo \
  "{servo_number: 9, pwm: 1500}"    # Hold position
```

### Mode Switching and Arming

```bash
# Switch to GUIDED mode
ros2 service call /dbvf/set_mode dbvf_msgs/srv/SetMode "{mode: 'GUIDED'}"
# Verify mode changes on Cube Orange (via Mission Planner or /dbvf/vehicle_state)

# Arm (props removed!)
ros2 service call /dbvf/arm_motors dbvf_msgs/srv/ArmMotors "{arm: true}"
# May need to disable pre-arm checks or satisfy them all first
```

---

## Summary: SITL vs Hardware Differences

| Aspect | SITL (Development) | Hardware (Jetson Orin) |
|--------|---------------------|------------------------|
| ArduPilot | SITL process on dev PC | Cube Orange flight controller |
| MAVLink transport | `udpin:0.0.0.0:14551` (UDP) | `/dev/ttyTHS1` at 921600 baud (UART) |
| Camera source | Gazebo plugin + ros_gz bridge | USB/CSI camera + ROS2 driver |
| Gazebo simulation | Required (iris_runway.launch.py) | Not used |
| Launch file | `precision_landing_sim.launch.py` | `precision_landing_hw.launch.py` |
| Config file | `sim_params.yaml` | `jetson_params.yaml` |
| GPS waypoints | Gazebo world coords (Canberra) | Real competition site coords |
| Camera calibration | Simulated (perfect intrinsics) | Physical calibration required |

### What Does NOT Change

The autonomy code itself requires **zero changes** — it is entirely config-driven:
- `precision_landing_node` — same FSM, same PID servo, same state transitions
- `tag_detector_adapter_node` — same debounce, same tag selection, same pose estimation
- `mission_sequencer_node` — same 17-state FSM, same service delegation
- `mavlink_interface_node` — same MAVLink protocol (just different transport)

---

## Quick-Reference: Minimal Steps to First Connection

1. Wire Cube Orange TELEM2 to Jetson Orin UART (TX→RX, RX→TX, GND→GND)
2. Set `SERIAL2_PROTOCOL=2`, `SERIAL2_BAUD=921` on Cube Orange
3. Add `baud_rate` parameter to `mavlink_interface_node.py` (see Section 4)
4. Update `jetson_params.yaml` with `connection_string: "/dev/ttyTHS1"` and `baud_rate: 921600`
5. `sudo usermod -aG dialout $USER` on the Jetson
6. Build workspace on Jetson: `colcon build --packages-select dbvf_msgs dbvf_autonomy`
7. Launch: `ros2 launch dbvf_autonomy precision_landing_hw.launch.py`
8. Verify: `ros2 topic echo /dbvf/vehicle_state`
