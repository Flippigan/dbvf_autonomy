"""Mission sequencer node — ROS2 wrapper around MissionStateMachine."""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from dbvf_msgs.msg import VehicleState
from dbvf_msgs.srv import (
    SetMode, ArmMotors, SendGuidedPosition, StartPrecisionLanding,
    StartMission, ResumeMission, AbortMission, DoSetServo, Takeoff,
)

from dbvf_autonomy.mission_state_machine import MissionStateMachine, MissionState, get_mission_phase
from dbvf_autonomy.mission_helpers import ft_to_m, validate_mission_config


class MissionSequencerNode(Node):
    def __init__(self):
        super().__init__('mission_sequencer')

        # Declare all parameters with defaults
        param_defaults = {
            'home_lat': -35.3632621,
            'home_lon': 149.1652374,
            'landing_lat': -35.3640000,
            'landing_lon': 149.1652374,
            'wa_lat': -35.3632531,
            'wa_lon': 149.1657896,
            'f1_lat': -35.3650000,
            'f1_lon': 149.1652374,
            'f2_lat': -35.3660000,
            'f2_lon': 149.1652374,
            'transit_altitude_ft': 35.0,
            'position_tolerance_m': 3.0,
            'takeoff_complete_alt_ft': 33.0,
            'drop_servo_number': 9,
            'drop_servo_pwm_release': 1100,
            'drop_servo_pwm_hold': 1500,
            'drop_settle_time_s': 2.0,
            'pickup_servo_number': 1,
            'pickup_servo_pwm_release': 1100,
            'pickup_servo_pwm_pickup': 1500,
            'pickup_settle_time_s': 2.0,
            'drop_target': 'F1',
            'mission_timeout_s': 540.0,
            'heartbeat_loss_timeout_s': 5.0,
            'service_call_timeout_s': 5.0,
            'guided_resend_interval_s': 1.0,
            'prefer_rangefinder': True,
            'rangefinder_max_m': 30.0,
            'wa_offset_forward': 0.0,
            'wa_offset_right': 0.0,
        }
        for name, default in param_defaults.items():
            self.declare_parameter(name, default)

        # Build config dict from parameters
        self.mission_config = {
            name: self.get_parameter(name).value for name in param_defaults
        }

        # Validate config
        errors = validate_mission_config(self.mission_config)
        if errors:
            for e in errors:
                self.get_logger().error(f'Config error: {e}')

        self.fsm = MissionStateMachine(self.mission_config)
        self.latest_vehicle_state = None
        self._last_heartbeat_time = time.time()
        self._prev_phase = 'IDLE'

        # Precompute transit altitude in meters
        self._transit_alt_m = ft_to_m(self.mission_config['transit_altitude_ft'])

        # Subscribers
        self.create_subscription(
            VehicleState, '/dbvf/vehicle_state', self._vehicle_state_cb, 10)
        self.create_subscription(
            Bool, '/dbvf/heartbeat_status', self._heartbeat_cb, 10)
        self.create_subscription(
            String, '/dbvf/landing_state', self._landing_state_cb, 10)

        # Publishers
        self.state_pub = self.create_publisher(String, '/dbvf/mission_state', 10)
        self.phase_pub = self.create_publisher(String, '/dbvf/mission_phase', 10)

        # Service clients
        self.set_mode_cli = self.create_client(SetMode, '/dbvf/set_mode')
        self.arm_cli = self.create_client(ArmMotors, '/dbvf/arm_motors')
        self.guided_cli = self.create_client(
            SendGuidedPosition, '/dbvf/send_guided_position')
        self.precision_land_cli = self.create_client(
            StartPrecisionLanding, '/dbvf/start_precision_landing')
        self.servo_cli = self.create_client(DoSetServo, '/dbvf/do_set_servo')
        self.takeoff_cli = self.create_client(Takeoff, '/dbvf/takeoff')
        self.arduino_servo_cli = self.create_client(
            DoSetServo, '/dbvf/arduino/set_servo')

        # Service servers
        self.create_service(
            StartMission, '/dbvf/start_mission', self._start_mission_cb)
        self.create_service(
            ResumeMission, '/dbvf/resume_mission', self._resume_mission_cb)
        self.create_service(
            AbortMission, '/dbvf/abort_mission', self._abort_mission_cb)

        # 10 Hz control loop
        self.create_timer(0.1, self._control_loop)
        self.get_logger().info('Mission sequencer node started')

    # -- Subscriber callbacks -------------------------------------------------

    def _vehicle_state_cb(self, msg):
        self.latest_vehicle_state = msg

    def _heartbeat_cb(self, msg):
        if msg.data:
            self._last_heartbeat_time = time.time()

    def _landing_state_cb(self, msg):
        self.fsm.set_landing_state(msg.data)

    # -- Service servers ------------------------------------------------------

    def _start_mission_cb(self, request, response):
        if self.fsm.state != MissionState.IDLE:
            response.success = False
            response.message = f'Already active: {self.fsm.state.value}'
            return response
        self.get_logger().info('Starting mission')
        self.fsm.start()
        response.success = True
        response.message = 'Mission started'
        return response

    def _resume_mission_cb(self, request, response):
        if self.fsm.state != MissionState.WAIT_FLAGGER:
            response.success = False
            response.message = f'Not in WAIT_FLAGGER: {self.fsm.state.value}'
            return response
        self.get_logger().info('Resuming mission from WAIT_FLAGGER')
        self.fsm.resume()
        response.success = True
        response.message = 'Resuming from WAIT_FLAGGER'
        return response

    def _abort_mission_cb(self, request, response):
        reason = request.reason or 'Manual abort'
        self.get_logger().warn(f'Abort requested: {reason}')
        self.fsm.abort(reason)
        self._call_set_mode('LAND')
        response.success = True
        response.message = f'Abort: {reason}'
        return response

    # -- Control loop ---------------------------------------------------------

    def _control_loop(self):
        if self.fsm.state == MissionState.IDLE:
            return

        vs = self.latest_vehicle_state
        if vs is None:
            return

        # Build a lightweight state object for the FSM — ROS2 messages
        # don't allow setting arbitrary attributes, so we wrap the fields.
        hb_timeout = self.mission_config['heartbeat_loss_timeout_s']

        class _VState:
            pass

        fs = _VState()
        fs.lat = vs.lat
        fs.lon = vs.lon
        fs.alt_rel = vs.alt_rel
        fs.armed = vs.armed
        fs.mode = vs.mode
        fs.vz = vs.vz
        fs.range_alt = vs.range_alt
        fs.heartbeat_ok = (time.time() - self._last_heartbeat_time) < hb_timeout

        now = time.time()
        prev_state = self.fsm.state

        state, info = self.fsm.update(fs, now)

        # Log state transitions
        if state != prev_state:
            self.get_logger().info(
                f'{prev_state.value} -> {state.value} ({info["action"]})')

        # Execute entry actions
        for action in info.get('entry_actions', []):
            self._execute_action(action)

        # Publish state
        state_msg = String()
        state_msg.data = state.value
        self.state_pub.publish(state_msg)

        # Publish phase on change
        phase = get_mission_phase(state)
        if phase != self._prev_phase:
            phase_msg = String()
            phase_msg.data = phase
            self.phase_pub.publish(phase_msg)
            self._prev_phase = phase

    # -- Action executor ------------------------------------------------------

    def _execute_action(self, action):
        cfg = self.mission_config
        alt = self._transit_alt_m

        if action == 'takeoff':
            self._call_takeoff(alt)
        elif action == 'set_mode_land':
            self._call_set_mode('LAND')
        elif action == 'set_mode_guided':
            self._call_set_mode('GUIDED')
        elif action == 'arm':
            self._call_arm(True)
        elif action == 'send_guided_position_h':
            self._call_guided_position(cfg['home_lat'], cfg['home_lon'], alt)
        elif action == 'send_guided_position_l':
            self._call_guided_position(cfg['landing_lat'], cfg['landing_lon'], alt)
        elif action == 'send_guided_position_l_alt':
            self._call_guided_position(cfg['landing_lat'], cfg['landing_lon'], alt)
        elif action == 'send_guided_position_drop':
            if cfg['drop_target'] == 'F2':
                self._call_guided_position(cfg['f2_lat'], cfg['f2_lon'], alt)
            else:
                self._call_guided_position(cfg['f1_lat'], cfg['f1_lon'], alt)
        elif action == 'send_guided_position_wa':
            self._call_guided_position(cfg['wa_lat'], cfg['wa_lon'], alt)
        elif action == 'send_guided_position_wa_alt':
            self._call_guided_position(cfg['wa_lat'], cfg['wa_lon'], alt)
        elif action == 'start_precision_landing':
            self._call_start_precision_landing(
                cfg['wa_lat'], cfg['wa_lon'],
                cfg.get('wa_offset_forward', 0.0),
                cfg.get('wa_offset_right', 0.0),
                cfg.get('wa_target_yaw', 0.0))
        elif action == 'servo_release':
            self._call_set_servo(
                cfg['drop_servo_number'], cfg['drop_servo_pwm_release'])
        elif action == 'arduino_servo_release':
            self._call_arduino_servo(
                cfg['pickup_servo_number'], cfg['pickup_servo_pwm_release'])
        elif action == 'arduino_servo_pickup':
            self._call_arduino_servo(
                cfg['pickup_servo_number'], cfg['pickup_servo_pwm_pickup'])

    # -- Service call helpers -------------------------------------------------

    def _call_takeoff(self, altitude):
        if not self.takeoff_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('takeoff service unavailable')
            return
        req = Takeoff.Request()
        req.altitude = altitude
        future = self.takeoff_cli.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'Takeoff: {f.result().message}') if f.result() else None)

    def _call_set_mode(self, mode):
        if not self.set_mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('set_mode service unavailable')
            return
        req = SetMode.Request()
        req.mode = mode
        future = self.set_mode_cli.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'Mode: {f.result().message}') if f.result() else None)

    def _call_arm(self, arm):
        if not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('arm_motors service unavailable')
            return
        req = ArmMotors.Request()
        req.arm = arm
        future = self.arm_cli.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'Arm: {f.result().message}') if f.result() else None)

    def _call_guided_position(self, lat, lon, alt):
        if not self.guided_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('send_guided_position service unavailable')
            return
        req = SendGuidedPosition.Request()
        req.lat = lat
        req.lon = lon
        req.alt = alt
        self.guided_cli.call_async(req)

    def _call_start_precision_landing(self, lat, lon,
                                      offset_forward=0.0, offset_right=0.0,
                                      target_yaw=0.0):
        if not self.precision_land_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('start_precision_landing service unavailable')
            return
        req = StartPrecisionLanding.Request()
        req.target_lat = lat
        req.target_lon = lon
        req.offset_forward = offset_forward
        req.offset_right = offset_right
        req.target_yaw = target_yaw
        future = self.precision_land_cli.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'Precision landing: {f.result().message}') if f.result() else None)

    def _call_set_servo(self, servo_number, pwm):
        if not self.servo_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('do_set_servo service unavailable')
            return
        req = DoSetServo.Request()
        req.servo_number = servo_number
        req.pwm = pwm
        future = self.servo_cli.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'Servo: {f.result().message}') if f.result() else None)

    def _call_arduino_servo(self, servo_number, pwm):
        if not self.arduino_servo_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('arduino/set_servo service unavailable')
            return
        req = DoSetServo.Request()
        req.servo_number = servo_number
        req.pwm = pwm
        future = self.arduino_servo_cli.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'Arduino servo: {f.result().message}') if f.result() else None)


def main():
    rclpy.init()
    node = MissionSequencerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
