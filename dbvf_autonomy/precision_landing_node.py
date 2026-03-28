"""Precision landing node -- state machine orchestrating the landing sequence."""
import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from dbvf_msgs.msg import LandingTargetPose, TagStatus, VehicleState
from dbvf_msgs.srv import SetMode, SendGuidedPosition, SendGuidedVelocity, StartPrecisionLanding


# ---------------------------------------------------------------------------
# PID controller (tested independently of ROS2)
# ---------------------------------------------------------------------------

class PIDController:
    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self._integral = 0.0
        self._prev_error = 0.0

    def update(self, error, dt):
        """Returns control output clamped to [-output_limit, output_limit]."""
        if dt <= 0.0:
            return 0.0
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt
        self._prev_error = error
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self):
        """Reset integral and previous error."""
        self._integral = 0.0
        self._prev_error = 0.0


# ---------------------------------------------------------------------------
# State machine (tested independently of ROS2)
# ---------------------------------------------------------------------------

class LandingState(Enum):
    IDLE = 'IDLE'
    APPROACH = 'APPROACH'
    SEARCH = 'SEARCH'
    DESCEND_COARSE = 'DESCEND_COARSE'
    DESCEND_HOLD = 'DESCEND_HOLD'
    DESCEND_OFFSET = 'DESCEND_OFFSET'
    DESCEND_FINAL = 'DESCEND_FINAL'
    SMALL_TAG_SEARCH = 'SMALL_TAG_SEARCH'
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

        # DESCEND_COARSE tracking
        self.small_tag_first_seen = None

        # DESCEND_HOLD tracking
        self.hold_start_time = None

        # SMALL_TAG_SEARCH tracking
        # Directions: 0=forward, 1=right, 2=backward, 3=left
        self.search_direction = 0
        self.search_phase = 'excursion'  # 'excursion' or 'return'
        self.search_phase_start = None

    def start(self, target_lat, target_lon):
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.state = LandingState.APPROACH
        self.tag_confirm_count = 0
        self.tag_lost_time = None
        self.start_time = None
        self.small_tag_first_seen = None
        self.hold_start_time = None
        self.search_direction = 0
        self.search_phase = 'excursion'
        self.search_phase_start = None

    def update(self, vehicle_state, tag_status, current_time):
        """Advance the state machine. Returns (state, action_dict).

        action_dict keys:
          'action': str describing what happened
          'vx', 'vy', 'vz': velocity commands (for DESCEND_* / SMALL_TAG_SEARCH)
          'use_offset': bool — whether PID target should include offset
          'tag_error_x', 'tag_error_y': body-frame position of tag (for PID)
        """
        if self.state == LandingState.IDLE:
            return self.state, {'action': None}
        if self.start_time is None:
            self.start_time = current_time
        if current_time - self.start_time > self.config['landing_timeout']:
            self.state = LandingState.ABORT_LAND
            return self.state, {'action': 'timeout'}

        if self.state == LandingState.APPROACH:
            return self._approach(vehicle_state)
        if self.state == LandingState.SEARCH:
            return self._search(vehicle_state, tag_status, current_time)
        if self.state == LandingState.DESCEND_COARSE:
            return self._descend_coarse(vehicle_state, tag_status, current_time)
        if self.state == LandingState.DESCEND_HOLD:
            return self._descend_hold(vehicle_state, tag_status, current_time)
        if self.state == LandingState.DESCEND_OFFSET:
            return self._descend_offset(vehicle_state, tag_status, current_time)
        if self.state == LandingState.DESCEND_FINAL:
            return self._descend_final(vehicle_state, tag_status, current_time)
        if self.state == LandingState.SMALL_TAG_SEARCH:
            return self._small_tag_search(vehicle_state, tag_status, current_time)
        if self.state == LandingState.ABORT_LAND:
            return self._abort(vehicle_state)
        if self.state == LandingState.LANDED:
            return self.state, {'action': None}
        return self.state, {'action': None}

    # -- Private state handlers -----------------------------------------------

    def _approach(self, vs):
        dist = self._lateral_distance(
            vs.lat, vs.lon, self.target_lat, self.target_lon)
        if (dist < self.config['position_tolerance']
                and vs.alt_rel <= self.config['approach_altitude'] + 0.5):
            self.state = LandingState.SEARCH
            return self.state, {'action': 'approach_complete'}
        return self.state, {'action': 'approaching'}

    def _search(self, vs, tag_status, current_time):
        if tag_status and tag_status.detected:
            self.tag_confirm_count += 1
            if self.tag_confirm_count >= self.config['tag_confirm_frames']:
                self.tag_confirm_count = 0
                self.tag_lost_time = None
                self.small_tag_first_seen = None
                self.state = LandingState.DESCEND_COARSE
                return self.state, {'action': 'tag_confirmed'}
        else:
            self.tag_confirm_count = 0

        if vs.alt_rel < self.config['min_search_altitude']:
            self.state = LandingState.ABORT_LAND
            return self.state, {'action': 'below_min_alt'}
        return self.state, {'action': 'searching'}

    def _descend_coarse(self, vs, tag_status, current_time):
        if self._is_landed(vs):
            self.state = LandingState.LANDED
            return self.state, {'action': 'landed'}

        tag_detected = tag_status and tag_status.detected
        small_tag_detected = (tag_detected
                              and tag_status.active_tag_id == self.config.get('secondary_tag_id', 1))

        # Track small tag continuous detection
        if small_tag_detected:
            if self.small_tag_first_seen is None:
                self.small_tag_first_seen = current_time
            elif (current_time - self.small_tag_first_seen
                  >= self.config['small_tag_confirm_time']):
                self.state = LandingState.DESCEND_HOLD
                self.hold_start_time = current_time
                self.small_tag_first_seen = None
                return self.state, {'action': 'small_tag_confirmed'}
        else:
            self.small_tag_first_seen = None

        # Tag lost tracking
        if not tag_detected:
            if self.tag_lost_time is None:
                self.tag_lost_time = current_time
            elif (current_time - self.tag_lost_time
                  > self.config['tag_lost_timeout']):
                self.tag_lost_time = None
                self.state = LandingState.SEARCH
                return self.state, {'action': 'tag_lost'}
        else:
            self.tag_lost_time = None

        # Floor altitude — trigger small tag search
        if (vs.alt_rel <= self.config['descend_floor_altitude']
                and not small_tag_detected):
            self.state = LandingState.SMALL_TAG_SEARCH
            self.search_direction = 0
            self.search_phase = 'excursion'
            self.search_phase_start = current_time
            return self.state, {'action': 'floor_altitude'}

        return self.state, {
            'action': 'descending',
            'vz': self.config['search_descent_rate'],
        }

    def _descend_hold(self, vs, tag_status, current_time):
        if self._is_landed(vs):
            self.state = LandingState.LANDED
            return self.state, {'action': 'landed'}

        if (current_time - self.hold_start_time
                >= self.config['hold_stabilize_time']):
            self.state = LandingState.DESCEND_OFFSET
            return self.state, {'action': 'hold_complete'}

        return self.state, {'action': 'holding', 'vz': 0.0}

    def _descend_offset(self, vs, tag_status, current_time):
        if self._is_landed(vs):
            self.state = LandingState.LANDED
            return self.state, {'action': 'landed'}

        tag_detected = tag_status and tag_status.detected
        small_tag_detected = (tag_detected
                              and tag_status.active_tag_id == self.config.get('secondary_tag_id', 1))

        # Tag lost tracking
        if not small_tag_detected:
            if self.tag_lost_time is None:
                self.tag_lost_time = current_time
            elif (current_time - self.tag_lost_time
                  > self.config['tag_lost_timeout']):
                self.tag_lost_time = None
                self.state = LandingState.SEARCH
                return self.state, {'action': 'tag_lost'}
        else:
            self.tag_lost_time = None

        return self.state, {
            'action': 'offsetting',
            'vz': 0.0,
            'use_offset': True,
        }

    def _descend_final(self, vs, tag_status, current_time):
        if self._is_landed(vs):
            self.state = LandingState.LANDED
            return self.state, {'action': 'landed'}

        tag_detected = tag_status and tag_status.detected
        small_tag_detected = (tag_detected
                              and tag_status.active_tag_id == self.config.get('secondary_tag_id', 1))

        return self.state, {
            'action': 'final_descent',
            'vz': self.config['final_descent_rate'],
            'use_offset': small_tag_detected,
        }

    def _small_tag_search(self, vs, tag_status, current_time):
        tag_detected = tag_status and tag_status.detected
        small_tag_detected = (tag_detected
                              and tag_status.active_tag_id == self.config.get('secondary_tag_id', 1))

        if small_tag_detected:
            self.state = LandingState.DESCEND_HOLD
            self.hold_start_time = current_time
            return self.state, {'action': 'small_tag_found'}

        radius = self.config['small_tag_search_radius']
        speed = self.config['small_tag_search_speed']
        phase_duration = radius / speed if speed > 0 else 1.0
        elapsed = current_time - self.search_phase_start

        if elapsed >= phase_duration:
            if self.search_phase == 'excursion':
                self.search_phase = 'return'
                self.search_phase_start = current_time
            else:
                # Completed return, advance to next direction
                self.search_direction += 1
                if self.search_direction >= 4:
                    # Full cycle complete, give up
                    self.state = LandingState.DESCEND_FINAL
                    return self.state, {'action': 'search_exhausted'}
                self.search_phase = 'excursion'
                self.search_phase_start = current_time

        # Compute velocity for current search movement
        # Directions: 0=forward(+x), 1=right(+y), 2=backward(-x), 3=left(-y)
        direction_vectors = [
            (1.0, 0.0),   # forward
            (0.0, 1.0),   # right
            (-1.0, 0.0),  # backward
            (0.0, -1.0),  # left
        ]
        dx, dy = direction_vectors[self.search_direction]
        if self.search_phase == 'return':
            dx, dy = -dx, -dy
        vx = dx * speed
        vy = dy * speed

        return self.state, {
            'action': 'searching',
            'vx': vx,
            'vy': vy,
            'vz': 0.0,
        }

    def _abort(self, vs):
        if self._is_landed(vs):
            self.state = LandingState.LANDED
            return self.state, {'action': 'landed'}
        return self.state, {'action': 'aborting'}

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

        # Existing parameters
        self.declare_parameter('approach_altitude', 8.0)
        self.declare_parameter('min_search_altitude', 1.0)
        self.declare_parameter('search_descent_rate', 0.3)
        self.declare_parameter('position_tolerance', 2.0)
        self.declare_parameter('tag_confirm_frames', 5)
        self.declare_parameter('tag_lost_timeout', 4.0)
        self.declare_parameter('landing_timeout', 60.0)

        # New parameters
        self.declare_parameter('descend_floor_altitude', 1.5)
        self.declare_parameter('final_descent_rate', 0.15)
        self.declare_parameter('small_tag_confirm_time', 2.0)
        self.declare_parameter('hold_stabilize_time', 1.0)
        self.declare_parameter('offset_forward', 0.0)
        self.declare_parameter('offset_right', 0.0)
        self.declare_parameter('offset_tolerance', 0.05)
        self.declare_parameter('small_tag_search_radius', 0.5)
        self.declare_parameter('small_tag_search_speed', 0.2)
        self.declare_parameter('servo_pid_p', 0.5)
        self.declare_parameter('servo_pid_i', 0.0)
        self.declare_parameter('servo_pid_d', 0.1)
        self.declare_parameter('servo_max_speed', 0.5)
        self.declare_parameter('secondary_tag_id', 1)

        config = {p: self.get_parameter(p).value for p in [
            'approach_altitude', 'min_search_altitude', 'search_descent_rate',
            'position_tolerance', 'tag_confirm_frames', 'tag_lost_timeout',
            'landing_timeout', 'descend_floor_altitude', 'final_descent_rate',
            'small_tag_confirm_time', 'hold_stabilize_time',
            'offset_forward', 'offset_right', 'offset_tolerance',
            'small_tag_search_radius', 'small_tag_search_speed',
            'secondary_tag_id']}

        self.fsm = LandingStateMachine(config)
        self.approach_alt = config['approach_altitude']
        self.search_descent_rate = config['search_descent_rate']
        self.offset_forward = config['offset_forward']
        self.offset_right = config['offset_right']
        self.offset_tolerance = config['offset_tolerance']

        # PID controllers for lateral servo
        kp = self.get_parameter('servo_pid_p').value
        ki = self.get_parameter('servo_pid_i').value
        kd = self.get_parameter('servo_pid_d').value
        max_speed = self.get_parameter('servo_max_speed').value
        self.pid_x = PIDController(kp, ki, kd, max_speed)
        self.pid_y = PIDController(kp, ki, kd, max_speed)

        self.latest_target = None
        self.latest_tag_status = None
        self.latest_vehicle_state = None
        self._search_target_alt = 0.0
        self._last_guided_time = 0.0
        self._last_control_time = 0.0

        # Subscribers
        self.create_subscription(
            LandingTargetPose, '/dbvf/landing_target_pose',
            self._target_cb, 10)
        self.create_subscription(
            TagStatus, '/dbvf/tag_status', self._tag_status_cb, 10)
        self.create_subscription(
            VehicleState, '/dbvf/vehicle_state', self._vehicle_state_cb, 10)

        # Publishers
        self.state_pub = self.create_publisher(
            String, '/dbvf/landing_state', 10)

        # Service clients
        self.set_mode_cli = self.create_client(SetMode, '/dbvf/set_mode')
        self.guided_cli = self.create_client(
            SendGuidedPosition, '/dbvf/send_guided_position')
        self.velocity_cli = self.create_client(
            SendGuidedVelocity, '/dbvf/send_guided_velocity')

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
        self.pid_x.reset()
        self.pid_y.reset()
        self._last_control_time = 0.0

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

        state, info = self.fsm.update(vs, self.latest_tag_status, now)

        # Handle transitions
        if state != prev_state:
            self.get_logger().info(
                f'{prev_state.value} -> {state.value} ({info["action"]})')

            if state == LandingState.SEARCH:
                self._call_set_mode('GUIDED')
                self._search_target_alt = vs.alt_rel
                self.pid_x.reset()
                self.pid_y.reset()
            elif state == LandingState.DESCEND_COARSE:
                self.pid_x.reset()
                self.pid_y.reset()
            elif state == LandingState.DESCEND_HOLD:
                self.pid_x.reset()
                self.pid_y.reset()
            elif state == LandingState.ABORT_LAND:
                self.get_logger().warn(f'Abort: {info["action"]}')
            elif state == LandingState.LANDED:
                self.get_logger().info('Landing complete')
                self.fsm.state = LandingState.IDLE

        # Continuous actions
        if state == LandingState.APPROACH:
            if now - self._last_guided_time >= 0.5:
                self._call_guided_position(
                    self.fsm.target_lat, self.fsm.target_lon,
                    self.approach_alt)
                self._last_guided_time = now

        elif state == LandingState.SEARCH:
            if self.latest_tag_status and self.latest_tag_status.detected:
                self._search_target_alt -= self.search_descent_rate * 0.05
                self._search_target_alt = max(
                    self._search_target_alt,
                    self.fsm.config['min_search_altitude'])
            if now - self._last_guided_time >= 0.5:
                self._call_guided_position(
                    self.fsm.target_lat, self.fsm.target_lon,
                    self._search_target_alt)
                self._last_guided_time = now

        elif state in (LandingState.DESCEND_COARSE, LandingState.DESCEND_HOLD,
                       LandingState.DESCEND_OFFSET, LandingState.DESCEND_FINAL):
            self._velocity_servo(state, info, now)

        elif state == LandingState.SMALL_TAG_SEARCH:
            # FSM provides velocity directly for search pattern
            vx = info.get('vx', 0.0)
            vy = info.get('vy', 0.0)
            vz = info.get('vz', 0.0)
            self._call_guided_velocity(vx, vy, vz)

        elif state == LandingState.ABORT_LAND:
            self._call_guided_velocity(
                0.0, 0.0, self.fsm.config['final_descent_rate'])

        # Publish current state
        msg = String()
        msg.data = state.value
        self.state_pub.publish(msg)

    def _velocity_servo(self, state, info, now):
        """PID-based lateral servo + state-determined vz."""
        dt = now - self._last_control_time if self._last_control_time > 0 else 0.05
        self._last_control_time = now

        vz = info.get('vz', 0.0)
        use_offset = info.get('use_offset', False)

        target = self.latest_target
        tag_detected = (self.latest_tag_status
                        and self.latest_tag_status.detected
                        and target is not None
                        and target.position_valid)

        if tag_detected:
            error_x = target.position_x
            error_y = target.position_y
            if use_offset:
                error_x -= self.offset_forward
                error_y -= self.offset_right

            # Check if offset is achieved (for DESCEND_OFFSET transition)
            if (state == LandingState.DESCEND_OFFSET
                    and math.sqrt(error_x**2 + error_y**2)
                    < self.offset_tolerance):
                self.fsm.state = LandingState.DESCEND_FINAL
                self.get_logger().info(
                    f'{state.value} -> DESCEND_FINAL (offset_achieved)')

            vx = self.pid_x.update(error_x, dt)
            vy = self.pid_y.update(error_y, dt)
        else:
            # Tag momentarily lost — hold position, reset PID
            vx = 0.0
            vy = 0.0
            self.pid_x.reset()
            self.pid_y.reset()

        self._call_guided_velocity(vx, vy, vz)

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

    def _call_guided_velocity(self, vx, vy, vz):
        if not self.velocity_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('send_guided_velocity service unavailable')
            return
        req = SendGuidedVelocity.Request()
        req.vx = vx
        req.vy = vy
        req.vz = vz
        self.velocity_cli.call_async(req)


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
