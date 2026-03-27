"""Precision landing node -- state machine orchestrating the landing sequence."""
import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from dbvf_msgs.msg import LandingTargetPose, TagStatus, VehicleState
from dbvf_msgs.srv import SetMode, SendGuidedPosition, StartPrecisionLanding


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
