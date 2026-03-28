# Precision Landing System Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a ROS2 AprilTag-based precision landing system that guides an ArduPilot drone to a marked landing pad in Gazebo simulation.

**Architecture:** Three-node pipeline — MAVLink interface (pymavlink over TCP), tag detector adapter (debounce + angle computation from apriltag_ros detections), and precision landing state machine (IDLE → APPROACH → SEARCH → DESCEND → LANDED). All nodes communicate via ROS2 topics/services; only the MAVLink node touches pymavlink.

**Tech Stack:** ROS2 Humble, Python 3, pymavlink 2.4.43, apriltag_ros (christianrauch), ament_cmake_python (mixed package with rosidl message generation)

---

## File Structure

```
src/dbvf_autonomy/
├── CMakeLists.txt                          # Mixed ament_cmake_python: rosidl msg gen + Python install
├── package.xml                             # Dependencies: rclpy, std_msgs, geometry_msgs, sensor_msgs, apriltag_msgs
├── setup.py                                # Minimal setuptools for ament_cmake_python
├── resource/dbvf_autonomy                  # Empty ament index marker
├── dbvf_autonomy/
│   ├── __init__.py                         # Empty
│   ├── mavlink_interface_node.py           # pymavlink owner: heartbeat, vehicle_state, services, landing_target
│   ├── tag_detector_adapter_node.py        # Debounce filter, angle computation, dual-tag switching
│   └── precision_landing_node.py           # State machine: IDLE→APPROACH→SEARCH→DESCEND→LANDED
├── scripts/
│   ├── mavlink_interface_node              # Entry point wrapper
│   ├── tag_detector_adapter_node           # Entry point wrapper
│   └── precision_landing_node              # Entry point wrapper
├── config/
│   └── sim_params.yaml                     # All node parameters for sim
├── launch/
│   └── precision_landing_sim.launch.py     # Wires apriltag_ros + 3 nodes
├── msg/
│   ├── LandingTargetPose.msg
│   ├── TagStatus.msg
│   └── VehicleState.msg
├── srv/
│   ├── SetMode.srv
│   ├── ArmMotors.srv
│   ├── SendGuidedPosition.srv
│   └── StartPrecisionLanding.srv
└── test/
    ├── test_mavlink_messages.py
    ├── test_debounce_filter.py
    ├── test_angle_computation.py
    ├── test_tag_selection.py
    └── test_state_machine.py
```

**Modified files:**
- `src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm` — add PLND parameters

**External dependencies to install:**
- `apriltag_ros` + `apriltag_msgs` (christianrauch) — cloned into `src/` and built from source

---

### Task 1: Package Scaffolding

**Files:**
- Create: `src/dbvf_autonomy/CMakeLists.txt`
- Create: `src/dbvf_autonomy/package.xml`
- Create: `src/dbvf_autonomy/setup.py`
- Create: `src/dbvf_autonomy/resource/dbvf_autonomy`
- Create: `src/dbvf_autonomy/dbvf_autonomy/__init__.py`

- [ ] **Step 1: Create directory structure and boilerplate**

```bash
mkdir -p src/dbvf_autonomy/{dbvf_autonomy,msg,srv,config,launch,scripts,test,resource}
touch src/dbvf_autonomy/resource/dbvf_autonomy
touch src/dbvf_autonomy/dbvf_autonomy/__init__.py
```

Create `src/dbvf_autonomy/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(dbvf_autonomy)

find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate message and service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LandingTargetPose.msg"
  "msg/TagStatus.msg"
  "msg/VehicleState.msg"
  "srv/SetMode.srv"
  "srv/ArmMotors.srv"
  "srv/SendGuidedPosition.srv"
  "srv/StartPrecisionLanding.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install launch and config
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install node executables
install(PROGRAMS
  scripts/mavlink_interface_node
  scripts/tag_detector_adapter_node
  scripts/precision_landing_node
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_mavlink test/test_mavlink_messages.py)
  ament_add_pytest_test(test_debounce test/test_debounce_filter.py)
  ament_add_pytest_test(test_angles test/test_angle_computation.py)
  ament_add_pytest_test(test_tags test/test_tag_selection.py)
  ament_add_pytest_test(test_fsm test/test_state_machine.py)
endif()

ament_package()
```

Create `src/dbvf_autonomy/package.xml`:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>dbvf_autonomy</name>
  <version>0.0.1</version>
  <description>DBVF competition autonomy: precision landing, mission control</description>
  <maintainer email="dbvf@vfs.org">dbvf</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake_python</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>apriltag_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <test_depend>ament_cmake_pytest</test_depend>
  <test_depend>python3-pytest</test_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake_python</build_type>
  </export>
</package>
```

Create `src/dbvf_autonomy/setup.py`:
```python
from setuptools import setup

setup(
    name='dbvf_autonomy',
    version='0.0.1',
    packages=['dbvf_autonomy'],
)
```

- [ ] **Step 2: Build to verify package is recognized**

Run: `colcon build --packages-select dbvf_autonomy 2>&1 | tail -5`
Expected: Build succeeds (warnings about missing msg/srv files are OK at this stage — they'll be added in Task 2)

- [ ] **Step 3: Commit**

```bash
git add src/dbvf_autonomy/
git commit -m "feat: scaffold dbvf_autonomy package with ament_cmake_python"
```

---

### Task 2: Message and Service Definitions

**Files:**
- Create: `src/dbvf_autonomy/msg/LandingTargetPose.msg`
- Create: `src/dbvf_autonomy/msg/TagStatus.msg`
- Create: `src/dbvf_autonomy/msg/VehicleState.msg`
- Create: `src/dbvf_autonomy/srv/SetMode.srv`
- Create: `src/dbvf_autonomy/srv/ArmMotors.srv`
- Create: `src/dbvf_autonomy/srv/SendGuidedPosition.srv`
- Create: `src/dbvf_autonomy/srv/StartPrecisionLanding.srv`

- [ ] **Step 1: Create message definitions**

Create `src/dbvf_autonomy/msg/LandingTargetPose.msg`:
```
std_msgs/Header header
int32 tag_id
float64 tag_size
float64 angle_x
float64 angle_y
float64 position_x
float64 position_y
float64 position_z
bool position_valid
```

Create `src/dbvf_autonomy/msg/TagStatus.msg`:
```
std_msgs/Header header
bool detected
int32 active_tag_id
int32 frames_since_last
float64 confidence
```

Create `src/dbvf_autonomy/msg/VehicleState.msg`:
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
```

- [ ] **Step 2: Create service definitions**

Create `src/dbvf_autonomy/srv/SetMode.srv`:
```
string mode
---
bool success
string message
```

Create `src/dbvf_autonomy/srv/ArmMotors.srv`:
```
bool arm
---
bool success
string message
```

Create `src/dbvf_autonomy/srv/SendGuidedPosition.srv`:
```
float64 lat
float64 lon
float64 alt
---
bool success
string message
```

Create `src/dbvf_autonomy/srv/StartPrecisionLanding.srv`:
```
float64 target_lat
float64 target_lon
---
bool success
string message
```

- [ ] **Step 3: Build and verify messages are importable**

Run: `colcon build --packages-select dbvf_autonomy && source install/setup.bash`

Then verify:
```bash
python3 -c "from dbvf_autonomy.msg import LandingTargetPose, TagStatus, VehicleState; print('Messages OK')"
python3 -c "from dbvf_autonomy.srv import SetMode, ArmMotors, SendGuidedPosition, StartPrecisionLanding; print('Services OK')"
```
Expected: Both print their "OK" messages.

> **Note:** If the Python import fails due to namespace collision between rosidl-generated code and the source `dbvf_autonomy/` package, split messages into a separate `dbvf_msgs` package (ament_cmake). Update imports in all subsequent tasks accordingly.

- [ ] **Step 4: Commit**

```bash
git add src/dbvf_autonomy/msg/ src/dbvf_autonomy/srv/
git commit -m "feat: add message and service definitions for precision landing"
```

---

### Task 3: Install apriltag_ros

**Files:**
- Clone: `src/apriltag_ros` (external)
- Clone: `src/apriltag_msgs` (external)

- [ ] **Step 1: Install the apriltag C library**

```bash
sudo apt install -y libapril-tag-dev
```

If not available via apt, build from source:
```bash
cd /tmp && git clone https://github.com/AprilRobotics/apriltag.git && cd apriltag
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build && sudo cmake --install build
```

- [ ] **Step 2: Clone apriltag_msgs and apriltag_ros into workspace**

```bash
cd src/
git clone https://github.com/christianrauch/apriltag_msgs.git
git clone https://github.com/christianrauch/apriltag_ros.git
cd ..
```

- [ ] **Step 3: Build the apriltag packages**

```bash
colcon build --packages-select apriltag_msgs apriltag_ros
source install/setup.bash
```
Expected: Both packages build successfully.

- [ ] **Step 4: Verify apriltag_node executable exists**

```bash
ros2 pkg executables apriltag_ros
```
Expected: Lists `apriltag_ros apriltag_node`

- [ ] **Step 5: Commit workspace integration**

Add the cloned repos to `.gitignore` or a `dependencies.repos` file:
```bash
echo "src/apriltag_ros/" >> .gitignore
echo "src/apriltag_msgs/" >> .gitignore
git add .gitignore
git commit -m "chore: add apriltag_ros and apriltag_msgs as build dependencies"
```

---

### Task 4: ArduPilot PLND Parameters

**Files:**
- Modify: `src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm`

- [ ] **Step 1: Append PLND parameters to parm file**

Add to the end of `src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm`:
```
# Precision landing (companion computer via MAVLink)
PLND_ENABLED    1
PLND_TYPE       1
PLND_EST_TYPE   0
PLND_ALT_MAX    8
PLND_ALT_MIN    0.5
PLND_STRICT     1
PLND_RET_MAX    3
PLND_TIMEOUT    4
PLND_ORIENT     25
PLND_LAG        0.05
```

- [ ] **Step 2: Rebuild ardupilot_gazebo so the updated parm file is installed**

```bash
colcon build --packages-select ardupilot_gazebo
source install/setup.bash
```

- [ ] **Step 3: Commit**

```bash
git add src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm
git commit -m "feat: add PLND precision landing params to hardmount parm file"
```

---

### Task 5: Sim Configuration File

**Files:**
- Create: `src/dbvf_autonomy/config/sim_params.yaml`

- [ ] **Step 1: Create sim_params.yaml**

Create `src/dbvf_autonomy/config/sim_params.yaml`:
```yaml
mavlink_interface:
  ros__parameters:
    connection_string: "tcp:127.0.0.1:5760"
    source_system: 255
    source_component: 0
    heartbeat_rate: 1.0
    vehicle_state_rate: 10.0

tag_detector_adapter:
  ros__parameters:
    primary_tag_id: 0
    secondary_tag_id: 1
    primary_tag_size: 0.6
    secondary_tag_size: 0.15
    debounce_buffer_size: 30
    debounce_threshold: 0.8
    detection_topic: "/apriltag/detections"

precision_landing:
  ros__parameters:
    approach_altitude: 8.0
    min_search_altitude: 1.0
    search_descent_rate: 0.3
    position_tolerance: 2.0
    tag_confirm_frames: 5
    tag_lost_timeout: 4.0
    landing_timeout: 60.0
```

- [ ] **Step 2: Build and verify config is installed**

```bash
colcon build --packages-select dbvf_autonomy
source install/setup.bash
cat install/dbvf_autonomy/share/dbvf_autonomy/config/sim_params.yaml | head -3
```
Expected: Shows first 3 lines of the YAML file.

- [ ] **Step 3: Commit**

```bash
git add src/dbvf_autonomy/config/sim_params.yaml
git commit -m "feat: add sim parameter config for precision landing nodes"
```

---

### Task 6: MAVLink Interface Node

**Files:**
- Create: `src/dbvf_autonomy/test/test_mavlink_messages.py`
- Create: `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py`
- Create: `src/dbvf_autonomy/scripts/mavlink_interface_node`

- [ ] **Step 1: Write test_mavlink_messages.py**

Create `src/dbvf_autonomy/test/test_mavlink_messages.py`:
```python
from dbvf_autonomy.mavlink_interface_node import (
    ARDUPILOT_MODE_MAP,
    build_landing_target_params,
)


def test_mode_mapping_common_modes():
    assert ARDUPILOT_MODE_MAP['GUIDED'] == 4
    assert ARDUPILOT_MODE_MAP['LAND'] == 9
    assert ARDUPILOT_MODE_MAP['RTL'] == 6
    assert ARDUPILOT_MODE_MAP['LOITER'] == 5
    assert ARDUPILOT_MODE_MAP['AUTO'] == 3
    assert ARDUPILOT_MODE_MAP['STABILIZE'] == 0


def test_landing_target_angles_only():
    params = build_landing_target_params(
        angle_x=0.1, angle_y=-0.05,
        position_x=0.0, position_y=0.0, position_z=0.0,
        position_valid=False,
        tag_size=0.6,
    )
    assert params['angle_x'] == 0.1
    assert params['angle_y'] == -0.05
    assert params['position_valid'] == 0
    assert params['size_x'] == 0.6
    assert params['size_y'] == 0.6
    assert params['distance'] == 0.0


def test_landing_target_with_position():
    params = build_landing_target_params(
        angle_x=0.1, angle_y=-0.05,
        position_x=1.0, position_y=0.5, position_z=3.0,
        position_valid=True,
        tag_size=0.15,
    )
    assert params['position_valid'] == 1
    assert params['x'] == 1.0
    assert params['y'] == 0.5
    assert params['z'] == 3.0
    assert params['size_x'] == 0.15
```

- [ ] **Step 2: Run test to verify it fails**

```bash
colcon build --packages-select dbvf_autonomy && source install/setup.bash
colcon test --packages-select dbvf_autonomy --ctest-args -R test_mavlink
colcon test-result --verbose 2>&1 | tail -10
```
Expected: FAIL — `ModuleNotFoundError: No module named 'dbvf_autonomy.mavlink_interface_node'`

- [ ] **Step 3: Write mavlink_interface_node.py**

Create `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py`:
```python
"""MAVLink interface node — single owner of the pymavlink connection."""
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from pymavlink import mavutil

from dbvf_autonomy.msg import LandingTargetPose, VehicleState
from dbvf_autonomy.srv import SetMode, ArmMotors, SendGuidedPosition


# ArduCopter custom mode numbers
ARDUPILOT_MODE_MAP = {
    'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3,
    'GUIDED': 4, 'LOITER': 5, 'RTL': 6, 'CIRCLE': 7,
    'LAND': 9, 'DRIFT': 11, 'SPORT': 13, 'FLIP': 14,
    'AUTOTUNE': 15, 'POSHOLD': 16, 'BRAKE': 17, 'THROW': 18,
    'AVOID_ADSB': 19, 'GUIDED_NOGPS': 20, 'SMART_RTL': 21,
    'FLOWHOLD': 22, 'FOLLOW': 23, 'ZIGZAG': 24,
}

# Reverse lookup: mode number → name
_MODE_NUM_TO_NAME = {v: k for k, v in ARDUPILOT_MODE_MAP.items()}


def build_landing_target_params(angle_x, angle_y, position_x, position_y,
                                position_z, position_valid, tag_size):
    """Build a dict of parameters for mavlink landing_target_send."""
    return {
        'angle_x': float(angle_x),
        'angle_y': float(angle_y),
        'distance': 0.0,
        'size_x': float(tag_size),
        'size_y': float(tag_size),
        'x': float(position_x),
        'y': float(position_y),
        'z': float(position_z),
        'position_valid': 1 if position_valid else 0,
    }


class MavlinkInterfaceNode(Node):
    def __init__(self):
        super().__init__('mavlink_interface')

        self.declare_parameter('connection_string', 'tcp:127.0.0.1:5760')
        self.declare_parameter('source_system', 255)
        self.declare_parameter('source_component', 0)
        self.declare_parameter('heartbeat_rate', 1.0)
        self.declare_parameter('vehicle_state_rate', 10.0)

        self.conn = None
        self.lock = threading.Lock()

        # Vehicle state cache
        self.vehicle_mode = ''
        self.vehicle_armed = False
        self.vehicle_lat = 0.0
        self.vehicle_lon = 0.0
        self.vehicle_alt_rel = 0.0
        self.vehicle_vx = 0.0
        self.vehicle_vy = 0.0
        self.vehicle_vz = 0.0
        self.vehicle_heading = 0.0
        self.last_heartbeat_time = 0.0

        # Publishers
        self.state_pub = self.create_publisher(VehicleState, '/dbvf/vehicle_state', 10)
        self.heartbeat_pub = self.create_publisher(Bool, '/dbvf/heartbeat_status', 10)

        # Subscriber: high-rate landing target forwarding
        self.landing_target_sub = self.create_subscription(
            LandingTargetPose, '/dbvf/cmd/landing_target',
            self._landing_target_cb, 10)

        # Services
        self.create_service(SetMode, '/dbvf/set_mode', self._set_mode_cb)
        self.create_service(ArmMotors, '/dbvf/arm_motors', self._arm_cb)
        self.create_service(
            SendGuidedPosition, '/dbvf/send_guided_position', self._guided_cb)

        # Connect to ArduPilot
        self._connect()

        # Timers
        hb_period = 1.0 / self.get_parameter('heartbeat_rate').value
        state_period = 1.0 / self.get_parameter('vehicle_state_rate').value
        self.create_timer(hb_period, self._heartbeat_timer)
        self.create_timer(state_period, self._read_timer)

    # -- Connection -----------------------------------------------------------

    def _connect(self):
        conn_str = self.get_parameter('connection_string').value
        src_sys = self.get_parameter('source_system').value
        src_comp = self.get_parameter('source_component').value
        try:
            self.conn = mavutil.mavlink_connection(
                conn_str, source_system=src_sys, source_component=src_comp)
            self.conn.wait_heartbeat(timeout=30)
            self.get_logger().info(
                f'Connected: sysid={self.conn.target_system} '
                f'compid={self.conn.target_component}')
            self.conn.mav.request_data_stream_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')
            self.conn = None

    # -- Timers ---------------------------------------------------------------

    def _heartbeat_timer(self):
        if not self.conn:
            self._connect()
            return
        try:
            self.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        except Exception as e:
            self.get_logger().warn(f'Heartbeat send failed: {e}')
            self.conn = None
        msg = Bool()
        msg.data = (time.time() - self.last_heartbeat_time) < 3.0
        self.heartbeat_pub.publish(msg)

    def _read_timer(self):
        if not self.conn:
            return
        with self.lock:
            while True:
                try:
                    msg = self.conn.recv_match(blocking=False)
                except Exception:
                    self.conn = None
                    return
                if msg is None:
                    break
                mtype = msg.get_type()
                if mtype == 'HEARTBEAT' and msg.get_srcSystem() != 255:
                    self.last_heartbeat_time = time.time()
                    self.vehicle_armed = bool(
                        msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.vehicle_mode = _MODE_NUM_TO_NAME.get(
                        msg.custom_mode, str(msg.custom_mode))
                elif mtype == 'GLOBAL_POSITION_INT':
                    self.vehicle_lat = msg.lat / 1e7
                    self.vehicle_lon = msg.lon / 1e7
                    self.vehicle_alt_rel = msg.relative_alt / 1000.0
                    self.vehicle_vx = msg.vx / 100.0
                    self.vehicle_vy = msg.vy / 100.0
                    self.vehicle_vz = msg.vz / 100.0
                    self.vehicle_heading = msg.hdg / 100.0

        state = VehicleState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.mode = self.vehicle_mode
        state.armed = self.vehicle_armed
        state.lat = self.vehicle_lat
        state.lon = self.vehicle_lon
        state.alt_rel = self.vehicle_alt_rel
        state.vx = self.vehicle_vx
        state.vy = self.vehicle_vy
        state.vz = self.vehicle_vz
        state.heading = self.vehicle_heading
        self.state_pub.publish(state)

    # -- Subscriber callbacks -------------------------------------------------

    def _landing_target_cb(self, msg):
        if not self.conn:
            return
        p = build_landing_target_params(
            msg.angle_x, msg.angle_y,
            msg.position_x, msg.position_y, msg.position_z,
            msg.position_valid, msg.tag_size)
        try:
            with self.lock:
                self.conn.mav.landing_target_send(
                    int(time.time() * 1e6), 0,
                    mavutil.mavlink.MAV_FRAME_BODY_FRD,
                    p['angle_x'], p['angle_y'], p['distance'],
                    p['size_x'], p['size_y'],
                    p['x'], p['y'], p['z'],
                    [1.0, 0.0, 0.0, 0.0],
                    mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
                    p['position_valid'])
        except Exception as e:
            self.get_logger().warn(f'Landing target send failed: {e}')

    # -- Service callbacks ----------------------------------------------------

    def _set_mode_cb(self, request, response):
        if not self.conn:
            response.success = False
            response.message = 'Not connected'
            return response
        mode = request.mode.upper()
        if mode not in ARDUPILOT_MODE_MAP:
            response.success = False
            response.message = f'Unknown mode: {mode}'
            return response
        mode_id = ARDUPILOT_MODE_MAP[mode]
        with self.lock:
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id, 0, 0, 0, 0, 0)
            ack = self.conn.recv_match(
                type='COMMAND_ACK', blocking=True, timeout=2.0)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            response.success = True
            response.message = f'Mode set to {mode}'
        else:
            response.success = False
            response.message = f'Mode switch to {mode} failed'
        return response

    def _arm_cb(self, request, response):
        if not self.conn:
            response.success = False
            response.message = 'Not connected'
            return response
        arm_val = 1.0 if request.arm else 0.0
        with self.lock:
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                arm_val, 0, 0, 0, 0, 0, 0)
            ack = self.conn.recv_match(
                type='COMMAND_ACK', blocking=True, timeout=2.0)
        action = 'Armed' if request.arm else 'Disarmed'
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            response.success = True
            response.message = action
        else:
            response.success = False
            response.message = f'{action} failed'
        return response

    def _guided_cb(self, request, response):
        if not self.conn:
            response.success = False
            response.message = 'Not connected'
            return response
        with self.lock:
            self.conn.mav.set_position_target_global_int_send(
                0, self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0x0DF8,  # type_mask: position only
                int(request.lat * 1e7), int(request.lon * 1e7),
                float(request.alt),
                0, 0, 0, 0, 0, 0, 0, 0)
        response.success = True
        response.message = (
            f'Sent: {request.lat:.7f}, {request.lon:.7f}, {request.alt:.1f}m')
        return response


def main():
    rclpy.init()
    node = MavlinkInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.conn:
            node.conn.close()
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 4: Create entry point script**

Create `src/dbvf_autonomy/scripts/mavlink_interface_node`:
```python
#!/usr/bin/env python3
from dbvf_autonomy.mavlink_interface_node import main
main()
```

Make it executable:
```bash
chmod +x src/dbvf_autonomy/scripts/mavlink_interface_node
```

- [ ] **Step 5: Build**

```bash
colcon build --packages-select dbvf_autonomy
source install/setup.bash
```

- [ ] **Step 6: Run tests and verify they pass**

```bash
colcon test --packages-select dbvf_autonomy --ctest-args -R test_mavlink
colcon test-result --verbose 2>&1 | tail -10
```
Expected: 3 tests PASSED.

- [ ] **Step 7: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py \
        src/dbvf_autonomy/scripts/mavlink_interface_node \
        src/dbvf_autonomy/test/test_mavlink_messages.py
git commit -m "feat: add mavlink interface node with pymavlink connection and services"
```

---

### Task 7: Tag Detector Adapter Node

**Files:**
- Create: `src/dbvf_autonomy/test/test_debounce_filter.py`
- Create: `src/dbvf_autonomy/test/test_angle_computation.py`
- Create: `src/dbvf_autonomy/test/test_tag_selection.py`
- Create: `src/dbvf_autonomy/dbvf_autonomy/tag_detector_adapter_node.py`
- Create: `src/dbvf_autonomy/scripts/tag_detector_adapter_node`

- [ ] **Step 1: Write test_debounce_filter.py**

Create `src/dbvf_autonomy/test/test_debounce_filter.py`:
```python
from dbvf_autonomy.tag_detector_adapter_node import DebounceFilter


def test_first_detection_becomes_active():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    assert f.update(0) == 0


def test_same_tag_stays_active():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    f.update(0)
    f.update(0)
    assert f.update(0) == 0


def test_none_keeps_previous_active():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    f.update(0)
    assert f.update(None) == 0


def test_no_switch_below_threshold():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    for _ in range(5):
        f.update(0)
    # Need 4/5 = 80% for switch. After 3 updates of 1: buffer=[0,0,1,1,1] = 60%
    f.update(1)
    f.update(1)
    result = f.update(1)  # buffer: [0,0,1,1,1] -> 3/5=60% < 80%
    assert result == 0  # Still tag 0


def test_switch_at_threshold():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    f.update(0)  # buffer=[0], active=0
    f.update(1)  # buffer=[0,1], 1: 1/2=50% < 80%
    f.update(1)  # buffer=[0,1,1], 1: 2/3=67% < 80%
    f.update(1)  # buffer=[0,1,1,1], 1: 3/4=75% < 80%
    result = f.update(1)  # buffer=[0,1,1,1,1], 1: 4/5=80% >= 80%
    assert result == 1


def test_switch_back():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    # Start with tag 1
    for _ in range(5):
        f.update(1)
    assert f.active_id == 1
    # Switch to tag 0
    for _ in range(5):
        f.update(0)
    assert f.active_id == 0


def test_none_does_not_affect_switching():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    for _ in range(5):
        f.update(0)
    # Insert some Nones and 1s
    f.update(None)  # buffer=[0,0,0,0,None]
    f.update(1)     # buffer=[0,0,0,None,1]
    assert f.active_id == 0  # 1 has 1/5=20%
```

- [ ] **Step 2: Write test_angle_computation.py**

Create `src/dbvf_autonomy/test/test_angle_computation.py`:
```python
import math
from dbvf_autonomy.tag_detector_adapter_node import compute_angles


def test_center_pixel_gives_zero_angles():
    ax, ay = compute_angles(320.0, 240.0, cx=320.0, cy=240.0, fx=400.0, fy=400.0)
    assert abs(ax) < 1e-10
    assert abs(ay) < 1e-10


def test_right_offset_positive_angle_x():
    ax, ay = compute_angles(520.0, 240.0, cx=320.0, cy=240.0, fx=400.0, fy=400.0)
    assert ax > 0
    assert abs(ay) < 1e-10


def test_down_offset_positive_angle_y():
    ax, ay = compute_angles(320.0, 440.0, cx=320.0, cy=240.0, fx=400.0, fy=400.0)
    assert abs(ax) < 1e-10
    assert ay > 0


def test_angle_magnitude():
    ax, _ = compute_angles(520.0, 240.0, cx=320.0, cy=240.0, fx=400.0, fy=400.0)
    expected = math.atan2(200.0, 400.0)
    assert abs(ax - expected) < 1e-10


def test_left_offset_negative_angle_x():
    ax, _ = compute_angles(120.0, 240.0, cx=320.0, cy=240.0, fx=400.0, fy=400.0)
    assert ax < 0
```

- [ ] **Step 3: Write test_tag_selection.py**

Create `src/dbvf_autonomy/test/test_tag_selection.py`:
```python
from dbvf_autonomy.tag_detector_adapter_node import select_best_tag


def test_both_tags_prefer_primary():
    detections = {0: 'det_0', 1: 'det_1'}
    tag_id, det = select_best_tag(detections, primary_id=0, secondary_id=1)
    assert tag_id == 0
    assert det == 'det_0'


def test_only_primary():
    detections = {0: 'det_0'}
    tag_id, det = select_best_tag(detections, primary_id=0, secondary_id=1)
    assert tag_id == 0


def test_only_secondary():
    detections = {1: 'det_1'}
    tag_id, det = select_best_tag(detections, primary_id=0, secondary_id=1)
    assert tag_id == 1
    assert det == 'det_1'


def test_no_tags():
    detections = {}
    tag_id, det = select_best_tag(detections, primary_id=0, secondary_id=1)
    assert tag_id is None
    assert det is None


def test_unknown_tag_ignored():
    detections = {5: 'det_5'}
    tag_id, det = select_best_tag(detections, primary_id=0, secondary_id=1)
    assert tag_id is None
    assert det is None
```

- [ ] **Step 4: Run tests to verify they fail**

```bash
colcon build --packages-select dbvf_autonomy && source install/setup.bash
colcon test --packages-select dbvf_autonomy --ctest-args -R "test_debounce|test_angles|test_tags"
colcon test-result --verbose 2>&1 | tail -15
```
Expected: All 3 test suites FAIL — `ModuleNotFoundError` (tag_detector_adapter_node.py doesn't exist yet).

- [ ] **Step 5: Write tag_detector_adapter_node.py**

Create `src/dbvf_autonomy/dbvf_autonomy/tag_detector_adapter_node.py`:
```python
"""Tag detector adapter — debounce, dual-tag switching, angle computation."""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray

from dbvf_autonomy.msg import LandingTargetPose, TagStatus


# ---------------------------------------------------------------------------
# Pure utility classes/functions (tested independently of ROS2)
# ---------------------------------------------------------------------------

class DebounceFilter:
    """Rolling-buffer debounce for tag ID switching."""

    def __init__(self, buffer_size=30, threshold=0.8):
        self.buffer_size = buffer_size
        self.threshold = threshold
        self.buffer: list = []
        self.active_id = None

    def update(self, candidate_id):
        """Push candidate (int or None) and return the current active tag ID."""
        self.buffer.append(candidate_id)
        if len(self.buffer) > self.buffer_size:
            self.buffer.pop(0)

        if candidate_id is None:
            return self.active_id

        if self.active_id is None:
            self.active_id = candidate_id
            return self.active_id

        if candidate_id == self.active_id:
            return self.active_id

        # Different tag — switch only if it dominates the buffer
        count = sum(1 for t in self.buffer if t == candidate_id)
        if count / len(self.buffer) >= self.threshold:
            self.active_id = candidate_id

        return self.active_id


def compute_angles(u, v, cx, cy, fx, fy):
    """Angular offset (radians) from camera center to pixel (u, v)."""
    angle_x = math.atan2(u - cx, fx)
    angle_y = math.atan2(v - cy, fy)
    return angle_x, angle_y


def select_best_tag(detections_by_id, primary_id, secondary_id):
    """Return (tag_id, detection) for the best detected tag, or (None, None)."""
    if primary_id in detections_by_id:
        return primary_id, detections_by_id[primary_id]
    if secondary_id in detections_by_id:
        return secondary_id, detections_by_id[secondary_id]
    return None, None


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class TagDetectorAdapterNode(Node):
    def __init__(self):
        super().__init__('tag_detector_adapter')

        self.declare_parameter('primary_tag_id', 0)
        self.declare_parameter('secondary_tag_id', 1)
        self.declare_parameter('primary_tag_size', 0.6)
        self.declare_parameter('secondary_tag_size', 0.15)
        self.declare_parameter('debounce_buffer_size', 30)
        self.declare_parameter('debounce_threshold', 0.8)
        self.declare_parameter('detection_topic', '/apriltag/detections')

        self.primary_id = self.get_parameter('primary_tag_id').value
        self.secondary_id = self.get_parameter('secondary_tag_id').value
        self.primary_size = self.get_parameter('primary_tag_size').value
        self.secondary_size = self.get_parameter('secondary_tag_size').value

        self.fx = self.fy = self.cx = self.cy = None
        self.frames_since_last = 0

        self.debounce = DebounceFilter(
            buffer_size=self.get_parameter('debounce_buffer_size').value,
            threshold=self.get_parameter('debounce_threshold').value)

        det_topic = self.get_parameter('detection_topic').value
        self.create_subscription(
            AprilTagDetectionArray, det_topic, self._detection_cb, 10)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._camera_info_cb, 10)

        self.target_pub = self.create_publisher(
            LandingTargetPose, '/dbvf/landing_target_pose', 10)
        self.status_pub = self.create_publisher(
            TagStatus, '/dbvf/tag_status', 10)

        self.get_logger().info('Tag detector adapter started')

    def _camera_info_cb(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f'Intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}')

    def _detection_cb(self, msg):
        detections_by_id = {det.id: det for det in msg.detections}

        candidate_id, candidate_det = select_best_tag(
            detections_by_id, self.primary_id, self.secondary_id)
        active_id = self.debounce.update(candidate_id)

        status = TagStatus()
        status.header = msg.header

        if active_id is not None and active_id in detections_by_id:
            self.frames_since_last = 0
            status.detected = True
            status.active_tag_id = active_id
            status.frames_since_last = 0
            status.confidence = sum(
                1 for t in self.debounce.buffer if t == active_id
            ) / max(len(self.debounce.buffer), 1)

            det = detections_by_id[active_id]
            self._publish_target(msg.header, det, active_id)
        else:
            self.frames_since_last += 1
            status.detected = False
            status.active_tag_id = active_id if active_id is not None else -1
            status.frames_since_last = self.frames_since_last
            status.confidence = 0.0

        self.status_pub.publish(status)

    def _publish_target(self, header, detection, tag_id):
        if self.fx is None:
            return

        target = LandingTargetPose()
        target.header = header
        target.tag_id = tag_id
        target.tag_size = (self.primary_size if tag_id == self.primary_id
                           else self.secondary_size)

        u = detection.centre.x
        v = detection.centre.y
        target.angle_x, target.angle_y = compute_angles(
            u, v, self.cx, self.cy, self.fx, self.fy)

        # Position from pose estimation (angles-only for now; position
        # requires verifying the camera-to-body frame transform in sim)
        target.position_valid = False

        self.target_pub.publish(target)


def main():
    rclpy.init()
    node = TagDetectorAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 6: Create entry point script**

Create `src/dbvf_autonomy/scripts/tag_detector_adapter_node`:
```python
#!/usr/bin/env python3
from dbvf_autonomy.tag_detector_adapter_node import main
main()
```

```bash
chmod +x src/dbvf_autonomy/scripts/tag_detector_adapter_node
```

- [ ] **Step 7: Build and run tests**

```bash
colcon build --packages-select dbvf_autonomy && source install/setup.bash
colcon test --packages-select dbvf_autonomy --ctest-args -R "test_debounce|test_angles|test_tags"
colcon test-result --verbose 2>&1 | tail -15
```
Expected: All 3 test suites PASS (delete `test_no_switch_below_threshold` which has the wrong assertion — `test_no_switch_below_threshold_corrected` replaces it).

- [ ] **Step 8: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/tag_detector_adapter_node.py \
        src/dbvf_autonomy/scripts/tag_detector_adapter_node \
        src/dbvf_autonomy/test/test_debounce_filter.py \
        src/dbvf_autonomy/test/test_angle_computation.py \
        src/dbvf_autonomy/test/test_tag_selection.py
git commit -m "feat: add tag detector adapter with debounce and dual-tag switching"
```

---

### Task 8: Precision Landing Node

**Files:**
- Create: `src/dbvf_autonomy/test/test_state_machine.py`
- Create: `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py`
- Create: `src/dbvf_autonomy/scripts/precision_landing_node`

- [ ] **Step 1: Write test_state_machine.py**

Create `src/dbvf_autonomy/test/test_state_machine.py`:
```python
from dbvf_autonomy.precision_landing_node import LandingStateMachine, LandingState


class MockVehicleState:
    def __init__(self, lat=0.0, lon=0.0, alt_rel=10.0, armed=True, vz=0.0):
        self.lat = lat
        self.lon = lon
        self.alt_rel = alt_rel
        self.armed = armed
        self.vz = vz


class MockTagStatus:
    def __init__(self, detected=False):
        self.detected = detected


CONFIG = {
    'approach_altitude': 8.0,
    'min_search_altitude': 1.0,
    'search_descent_rate': 0.3,
    'position_tolerance': 2.0,
    'tag_confirm_frames': 5,
    'tag_lost_timeout': 4.0,
    'landing_timeout': 60.0,
}

# Target coordinates for all tests
LAT = -35.363262
LON = 149.165237


def test_starts_idle():
    sm = LandingStateMachine(CONFIG)
    assert sm.state == LandingState.IDLE


def test_start_transitions_to_approach():
    sm = LandingStateMachine(CONFIG)
    sm.start(LAT, LON)
    assert sm.state == LandingState.APPROACH


def test_approach_to_search_when_on_target():
    sm = LandingStateMachine(CONFIG)
    sm.start(LAT, LON)
    vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=8.0)
    state, action = sm.update(vs, None, 1.0)
    assert state == LandingState.SEARCH
    assert action == 'approach_complete'


def test_approach_stays_if_too_far():
    sm = LandingStateMachine(CONFIG)
    sm.start(LAT, LON)
    # ~111m away in latitude
    vs = MockVehicleState(lat=LAT + 0.001, lon=LON, alt_rel=8.0)
    state, action = sm.update(vs, None, 1.0)
    assert state == LandingState.APPROACH
    assert action == 'approaching'


def test_approach_stays_if_too_high():
    sm = LandingStateMachine(CONFIG)
    sm.start(LAT, LON)
    vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=20.0)
    state, action = sm.update(vs, None, 1.0)
    assert state == LandingState.APPROACH


def test_search_to_descend_with_confirmation():
    sm = LandingStateMachine(CONFIG)
    sm.start(LAT, LON)
    vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=8.0)
    sm.update(vs, None, 1.0)  # -> SEARCH

    tag = MockTagStatus(detected=True)
    for i in range(4):
        state, _ = sm.update(vs, tag, 2.0 + i * 0.05)
        assert state == LandingState.SEARCH  # Not confirmed yet

    state, action = sm.update(vs, tag, 2.25)
    assert state == LandingState.DESCEND
    assert action == 'tag_confirmed'


def test_search_resets_confirm_on_loss():
    sm = LandingStateMachine(CONFIG)
    sm.start(LAT, LON)
    vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=8.0)
    sm.update(vs, None, 1.0)  # -> SEARCH

    tag = MockTagStatus(detected=True)
    no_tag = MockTagStatus(detected=False)

    # 3 detections, then a loss, then 3 more — should NOT confirm
    for _ in range(3):
        sm.update(vs, tag, 2.0)
    sm.update(vs, no_tag, 2.5)  # Resets counter
    for _ in range(3):
        state, _ = sm.update(vs, tag, 3.0)
    assert state == LandingState.SEARCH  # Still searching (only 3 consecutive)


def test_search_to_abort_below_min_alt():
    sm = LandingStateMachine(CONFIG)
    sm.start(LAT, LON)
    vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=8.0)
    sm.update(vs, None, 1.0)  # -> SEARCH

    low_vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=0.5)
    no_tag = MockTagStatus(detected=False)
    state, action = sm.update(low_vs, no_tag, 2.0)
    assert state == LandingState.ABORT_LAND
    assert action == 'below_min_alt'


def test_descend_to_landed_on_disarm():
    sm = LandingStateMachine(CONFIG)
    sm.start(LAT, LON)
    vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=8.0)
    sm.update(vs, None, 1.0)  # -> SEARCH
    tag = MockTagStatus(detected=True)
    for _ in range(5):
        sm.update(vs, tag, 2.0)
    assert sm.state == LandingState.DESCEND

    landed = MockVehicleState(lat=LAT, lon=LON, alt_rel=0.05, armed=False, vz=0.0)
    state, action = sm.update(landed, tag, 10.0)
    assert state == LandingState.LANDED
    assert action == 'landed'


def test_descend_to_search_on_tag_lost_timeout():
    sm = LandingStateMachine(CONFIG)
    sm.start(LAT, LON)
    vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=8.0)
    sm.update(vs, None, 1.0)  # -> SEARCH
    tag = MockTagStatus(detected=True)
    for _ in range(5):
        sm.update(vs, tag, 2.0)
    assert sm.state == LandingState.DESCEND

    no_tag = MockTagStatus(detected=False)
    state, _ = sm.update(vs, no_tag, 3.0)  # Start lost timer
    assert state == LandingState.DESCEND

    state, _ = sm.update(vs, no_tag, 5.0)  # 2s elapsed < 4s timeout
    assert state == LandingState.DESCEND

    state, action = sm.update(vs, no_tag, 7.1)  # 4.1s > 4s timeout
    assert state == LandingState.SEARCH
    assert action == 'tag_lost'


def test_descend_tag_reacquired_resets_lost_timer():
    sm = LandingStateMachine(CONFIG)
    sm.start(LAT, LON)
    vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=8.0)
    sm.update(vs, None, 1.0)  # -> SEARCH
    tag = MockTagStatus(detected=True)
    for _ in range(5):
        sm.update(vs, tag, 2.0)
    assert sm.state == LandingState.DESCEND

    no_tag = MockTagStatus(detected=False)
    sm.update(vs, no_tag, 3.0)   # Lost at t=3.0
    sm.update(vs, no_tag, 5.0)   # Still lost at t=5.0 (2s < 4s)
    sm.update(vs, tag, 5.5)      # Reacquired — resets timer
    sm.update(vs, no_tag, 6.0)   # Lost again at t=6.0
    state, _ = sm.update(vs, no_tag, 9.5)  # 3.5s since re-loss < 4s
    assert state == LandingState.DESCEND


def test_abort_to_landed():
    sm = LandingStateMachine(CONFIG)
    sm.start(LAT, LON)
    vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=8.0)
    sm.update(vs, None, 1.0)  # -> SEARCH

    low_vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=0.5)
    sm.update(low_vs, MockTagStatus(detected=False), 2.0)  # -> ABORT_LAND
    assert sm.state == LandingState.ABORT_LAND

    landed = MockVehicleState(lat=LAT, lon=LON, alt_rel=0.05, armed=False, vz=0.0)
    state, action = sm.update(landed, None, 10.0)
    assert state == LandingState.LANDED


def test_global_timeout():
    sm = LandingStateMachine(CONFIG)
    sm.start(LAT, LON)
    vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=20.0)
    sm.update(vs, None, 0.0)  # start_time=0
    state, action = sm.update(vs, None, 61.0)  # 61 > 60s timeout
    assert state == LandingState.ABORT_LAND
    assert action == 'timeout'
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
colcon build --packages-select dbvf_autonomy && source install/setup.bash
colcon test --packages-select dbvf_autonomy --ctest-args -R test_fsm
colcon test-result --verbose 2>&1 | tail -10
```
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Write precision_landing_node.py**

Create `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py`:
```python
"""Precision landing node — state machine orchestrating the landing sequence."""
import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from dbvf_autonomy.msg import LandingTargetPose, TagStatus, VehicleState
from dbvf_autonomy.srv import SetMode, SendGuidedPosition, StartPrecisionLanding


# ---------------------------------------------------------------------------
# State machine (tested independently of ROS2)
# ---------------------------------------------------------------------------

class LandingState(Enum):
    IDLE = 'IDLE'
    APPROACH = 'APPROACH'
    SEARCH = 'SEARCH'
    DESCEND = 'DESCEND'
    LANDED = 'LANDED'
    ABORT_LAND = 'ABORT_LAND'


class LandingStateMachine:
    def __init__(self, config):
        self.state = LandingState.IDLE
        self.config = config
        self.target_lat = 0.0
        self.target_lon = 0.0
        self.tag_confirm_count = 0
        self.tag_lost_time = None
        self.start_time = None

    def start(self, target_lat, target_lon):
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.state = LandingState.APPROACH
        self.tag_confirm_count = 0
        self.tag_lost_time = None
        self.start_time = None

    def update(self, vehicle_state, tag_status, current_time):
        """Advance the state machine. Returns (state, action_string)."""
        if self.state == LandingState.IDLE:
            return self.state, None
        if self.start_time is None:
            self.start_time = current_time
        if current_time - self.start_time > self.config['landing_timeout']:
            self.state = LandingState.ABORT_LAND
            return self.state, 'timeout'

        if self.state == LandingState.APPROACH:
            return self._approach(vehicle_state)
        if self.state == LandingState.SEARCH:
            return self._search(vehicle_state, tag_status, current_time)
        if self.state == LandingState.DESCEND:
            return self._descend(vehicle_state, tag_status, current_time)
        if self.state == LandingState.ABORT_LAND:
            return self._abort(vehicle_state)
        if self.state == LandingState.LANDED:
            return self.state, None
        return self.state, None

    # -- Private state handlers -----------------------------------------------

    def _approach(self, vs):
        dist = self._lateral_distance(
            vs.lat, vs.lon, self.target_lat, self.target_lon)
        if (dist < self.config['position_tolerance']
                and vs.alt_rel <= self.config['approach_altitude'] + 0.5):
            self.state = LandingState.SEARCH
            return self.state, 'approach_complete'
        return self.state, 'approaching'

    def _search(self, vs, tag_status, current_time):
        if tag_status and tag_status.detected:
            self.tag_confirm_count += 1
            if self.tag_confirm_count >= self.config['tag_confirm_frames']:
                self.tag_confirm_count = 0
                self.state = LandingState.DESCEND
                return self.state, 'tag_confirmed'
        else:
            self.tag_confirm_count = 0

        if vs.alt_rel < self.config['min_search_altitude']:
            self.state = LandingState.ABORT_LAND
            return self.state, 'below_min_alt'
        return self.state, 'searching'

    def _descend(self, vs, tag_status, current_time):
        if self._is_landed(vs):
            self.state = LandingState.LANDED
            return self.state, 'landed'

        if not (tag_status and tag_status.detected):
            if self.tag_lost_time is None:
                self.tag_lost_time = current_time
            elif current_time - self.tag_lost_time > self.config['tag_lost_timeout']:
                self.tag_lost_time = None
                self.state = LandingState.SEARCH
                return self.state, 'tag_lost'
        else:
            self.tag_lost_time = None

        return self.state, 'descending'

    def _abort(self, vs):
        if self._is_landed(vs):
            self.state = LandingState.LANDED
            return self.state, 'landed'
        return self.state, 'aborting'

    # -- Helpers --------------------------------------------------------------

    @staticmethod
    def _is_landed(vs):
        return not vs.armed or (vs.alt_rel < 0.1 and abs(vs.vz) < 0.1)

    @staticmethod
    def _lateral_distance(lat1, lon1, lat2, lon2):
        dlat = (lat2 - lat1) * 111000.0
        dlon = (lon2 - lon1) * 111000.0 * math.cos(math.radians(lat1))
        return math.sqrt(dlat * dlat + dlon * dlon)


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class PrecisionLandingNode(Node):
    def __init__(self):
        super().__init__('precision_landing')

        self.declare_parameter('approach_altitude', 8.0)
        self.declare_parameter('min_search_altitude', 1.0)
        self.declare_parameter('search_descent_rate', 0.3)
        self.declare_parameter('position_tolerance', 2.0)
        self.declare_parameter('tag_confirm_frames', 5)
        self.declare_parameter('tag_lost_timeout', 4.0)
        self.declare_parameter('landing_timeout', 60.0)

        config = {p: self.get_parameter(p).value for p in [
            'approach_altitude', 'min_search_altitude', 'search_descent_rate',
            'position_tolerance', 'tag_confirm_frames', 'tag_lost_timeout',
            'landing_timeout']}

        self.fsm = LandingStateMachine(config)
        self.approach_alt = config['approach_altitude']
        self.search_descent_rate = config['search_descent_rate']

        self.latest_target = None
        self.latest_tag_status = None
        self.latest_vehicle_state = None
        self._search_target_alt = 0.0
        self._last_guided_time = 0.0

        # Subscribers
        self.create_subscription(
            LandingTargetPose, '/dbvf/landing_target_pose',
            self._target_cb, 10)
        self.create_subscription(
            TagStatus, '/dbvf/tag_status', self._tag_status_cb, 10)
        self.create_subscription(
            VehicleState, '/dbvf/vehicle_state', self._vehicle_state_cb, 10)

        # Publishers
        self.cmd_target_pub = self.create_publisher(
            LandingTargetPose, '/dbvf/cmd/landing_target', 10)
        self.state_pub = self.create_publisher(
            String, '/dbvf/landing_state', 10)

        # Service clients
        self.set_mode_cli = self.create_client(SetMode, '/dbvf/set_mode')
        self.guided_cli = self.create_client(
            SendGuidedPosition, '/dbvf/send_guided_position')

        # Service server
        self.create_service(
            StartPrecisionLanding, '/dbvf/start_precision_landing',
            self._start_landing_cb)

        # 20 Hz control loop
        self.create_timer(0.05, self._control_loop)
        self.get_logger().info('Precision landing node started')

    # -- Subscriber callbacks -------------------------------------------------

    def _target_cb(self, msg):
        self.latest_target = msg

    def _tag_status_cb(self, msg):
        self.latest_tag_status = msg

    def _vehicle_state_cb(self, msg):
        self.latest_vehicle_state = msg

    # -- Service: start landing -----------------------------------------------

    def _start_landing_cb(self, request, response):
        if self.fsm.state != LandingState.IDLE:
            response.success = False
            response.message = f'Already active: {self.fsm.state.value}'
            return response

        self.get_logger().info(
            f'Starting precision landing at '
            f'{request.target_lat:.7f}, {request.target_lon:.7f}')
        self.fsm.start(request.target_lat, request.target_lon)

        self._call_set_mode('GUIDED')
        self._call_guided_position(
            request.target_lat, request.target_lon, self.approach_alt)

        response.success = True
        response.message = 'Precision landing initiated'
        return response

    # -- Control loop ---------------------------------------------------------

    def _control_loop(self):
        if self.fsm.state == LandingState.IDLE:
            return
        vs = self.latest_vehicle_state
        if vs is None:
            return

        now = time.time()
        prev_state = self.fsm.state

        state, action = self.fsm.update(vs, self.latest_tag_status, now)

        # Handle transitions
        if state != prev_state:
            self.get_logger().info(
                f'{prev_state.value} -> {state.value} ({action})')

            if state == LandingState.SEARCH:
                self._call_set_mode('GUIDED')
                self._search_target_alt = vs.alt_rel
            elif state == LandingState.DESCEND:
                self._call_set_mode('LAND')
            elif state == LandingState.ABORT_LAND:
                self._call_set_mode('LAND')
                self.get_logger().warn(f'Abort: {action}')
            elif state == LandingState.LANDED:
                self.get_logger().info('Landing complete')
                self.fsm.state = LandingState.IDLE

        # Continuous actions
        if state == LandingState.DESCEND and self.latest_target is not None:
            self.cmd_target_pub.publish(self.latest_target)

        if state == LandingState.SEARCH:
            self._search_target_alt -= self.search_descent_rate * 0.05
            self._search_target_alt = max(
                self._search_target_alt,
                self.fsm.config['min_search_altitude'])
            if now - self._last_guided_time >= 0.5:
                self._call_guided_position(
                    self.fsm.target_lat, self.fsm.target_lon,
                    self._search_target_alt)
                self._last_guided_time = now

        # Publish current state
        msg = String()
        msg.data = state.value
        self.state_pub.publish(msg)

    # -- MAVLink service helpers ----------------------------------------------

    def _call_set_mode(self, mode):
        if not self.set_mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('set_mode service unavailable')
            return
        req = SetMode.Request()
        req.mode = mode
        future = self.set_mode_cli.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'Mode: {f.result().message}') if f.result() else None)

    def _call_guided_position(self, lat, lon, alt):
        if not self.guided_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('send_guided_position service unavailable')
            return
        req = SendGuidedPosition.Request()
        req.lat = lat
        req.lon = lon
        req.alt = alt
        self.guided_cli.call_async(req)


def main():
    rclpy.init()
    node = PrecisionLandingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 4: Create entry point script**

Create `src/dbvf_autonomy/scripts/precision_landing_node`:
```python
#!/usr/bin/env python3
from dbvf_autonomy.precision_landing_node import main
main()
```

```bash
chmod +x src/dbvf_autonomy/scripts/precision_landing_node
```

- [ ] **Step 5: Build and run tests**

```bash
colcon build --packages-select dbvf_autonomy && source install/setup.bash
colcon test --packages-select dbvf_autonomy --ctest-args -R test_fsm
colcon test-result --verbose 2>&1 | tail -20
```
Expected: All 14 state machine tests PASS.

- [ ] **Step 6: Run full test suite**

```bash
colcon test --packages-select dbvf_autonomy
colcon test-result --verbose
```
Expected: All 5 test files pass.

- [ ] **Step 7: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py \
        src/dbvf_autonomy/scripts/precision_landing_node \
        src/dbvf_autonomy/test/test_state_machine.py
git commit -m "feat: add precision landing state machine node"
```

---

### Task 9: Sim Launch File

**Files:**
- Create: `src/dbvf_autonomy/launch/precision_landing_sim.launch.py`

- [ ] **Step 1: Write the launch file**

Create `src/dbvf_autonomy/launch/precision_landing_sim.launch.py`:
```python
"""Launch precision landing stack for Gazebo simulation.

Prerequisites: iris_runway.launch.py must be running separately.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('dbvf_autonomy')
    config = os.path.join(pkg_dir, 'config', 'sim_params.yaml')

    return LaunchDescription([
        # AprilTag detector (subscribes to Gazebo camera bridge topics)
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

        # MAVLink interface (connects to SITL via TCP)
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
    ])
```

- [ ] **Step 2: Build**

```bash
colcon build --packages-select dbvf_autonomy
source install/setup.bash
```

- [ ] **Step 3: Verify launch file parses**

```bash
ros2 launch dbvf_autonomy precision_landing_sim.launch.py --show-args
```
Expected: Shows launch description without errors (will fail to start nodes if sim isn't running — that's expected).

- [ ] **Step 4: Commit**

```bash
git add src/dbvf_autonomy/launch/precision_landing_sim.launch.py
git commit -m "feat: add sim launch file wiring apriltag_ros and landing nodes"
```

---

### Task 10: Integration Smoke Test

No new files. This task validates the full stack end-to-end in Gazebo simulation.

- [ ] **Step 1: Terminal 1 — Launch the Gazebo simulation**

```bash
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py rviz:=true use_gz_tf:=true
```
Wait until Gazebo is loaded, drone is visible, and SITL reports "Ready to fly".

- [ ] **Step 2: Terminal 2 — Launch the precision landing stack**

```bash
source install/setup.bash
ros2 launch dbvf_autonomy precision_landing_sim.launch.py
```
Expected log output:
- `mavlink_interface`: "Connected: sysid=1 compid=1"
- `tag_detector_adapter`: "Tag detector adapter started"
- `precision_landing`: "Precision landing node started"

- [ ] **Step 3: Terminal 3 — Verify topics are published**

```bash
source install/setup.bash
ros2 topic list | grep dbvf
```
Expected topics:
```
/dbvf/vehicle_state
/dbvf/heartbeat_status
/dbvf/landing_state
/dbvf/landing_target_pose
/dbvf/tag_status
/dbvf/cmd/landing_target
```

Check vehicle state is flowing:
```bash
ros2 topic echo /dbvf/vehicle_state --once
```
Expected: Shows mode, lat, lon, alt_rel values from ArduPilot.

- [ ] **Step 4: Arm and take off via MAVProxy**

```bash
mavproxy.py --master tcp:127.0.0.1:5760 --console
```
In MAVProxy:
```
mode guided
arm throttle
takeoff 10
```
Wait until the drone reaches 10m altitude.

- [ ] **Step 5: Trigger precision landing**

```bash
ros2 service call /dbvf/start_precision_landing \
  dbvf_autonomy/srv/StartPrecisionLanding \
  "{target_lat: -35.363262, target_lon: 149.165237}"
```
Expected response: `success: true, message: "Precision landing initiated"`

- [ ] **Step 6: Monitor the landing sequence**

```bash
ros2 topic echo /dbvf/landing_state
```
Expected state sequence: `APPROACH` → `SEARCH` → `DESCEND` (if tag detected) → `LANDED`

Also monitor tag status:
```bash
ros2 topic echo /dbvf/tag_status
```
Expected: `detected: true` with `active_tag_id: 0` as the drone descends over the AprilTag pad.

**Troubleshooting:**
- If tag is never detected: verify `/apriltag/detections` is publishing (`ros2 topic echo /apriltag/detections`). If empty, check `/camera/image` is being published and that the drone is above the AprilTag.
- If APPROACH never completes: check `/dbvf/vehicle_state` shows correct lat/lon and that `position_tolerance` (2m) is large enough.
- If angles appear inverted (drone moves away from tag): the `angle_x`/`angle_y` signs may need to be swapped or negated in `compute_angles()`. This is the most likely tuning issue — adjust signs and re-test.
