"""MAVLink interface node — single owner of the pymavlink connection."""
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from pymavlink import mavutil

from dbvf_msgs.msg import LandingTargetPose, VehicleState
from dbvf_msgs.srv import SetMode, ArmMotors, SendGuidedPosition


# ArduCopter custom mode numbers
ARDUPILOT_MODE_MAP = {
    'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3,
    'GUIDED': 4, 'LOITER': 5, 'RTL': 6, 'CIRCLE': 7,
    'LAND': 9, 'DRIFT': 11, 'SPORT': 13, 'FLIP': 14,
    'AUTOTUNE': 15, 'POSHOLD': 16, 'BRAKE': 17, 'THROW': 18,
    'AVOID_ADSB': 19, 'GUIDED_NOGPS': 20, 'SMART_RTL': 21,
    'FLOWHOLD': 22, 'FOLLOW': 23, 'ZIGZAG': 24,
}

# Reverse lookup: mode number -> name
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
