"""Mission sequencer state machine — pure Python, no ROS2 dependencies."""
from enum import Enum

from dbvf_autonomy.mission_helpers import ft_to_m, haversine_distance_m


class MissionState(Enum):
    IDLE = 'IDLE'
    PREFLIGHT_CHECK = 'PREFLIGHT_CHECK'
    TAKEOFF_H = 'TAKEOFF_H'
    TRANSIT_H_TO_L = 'TRANSIT_H_TO_L'
    LAND_L = 'LAND_L'
    WAIT_FLAGGER = 'WAIT_FLAGGER'
    TAKEOFF_L = 'TAKEOFF_L'
    TRANSIT_TO_DROP = 'TRANSIT_TO_DROP'
    DROP_PAYLOAD = 'DROP_PAYLOAD'
    TRANSIT_TO_WA = 'TRANSIT_TO_WA'
    LAND_WA = 'LAND_WA'
    TAKEOFF_WA = 'TAKEOFF_WA'
    TRANSIT_TO_DROP_2 = 'TRANSIT_TO_DROP_2'
    DROP_PAYLOAD_2 = 'DROP_PAYLOAD_2'
    TRANSIT_TO_H = 'TRANSIT_TO_H'
    LAND_H = 'LAND_H'
    COMPLETE = 'COMPLETE'
    ABORT = 'ABORT'


# States grouped by mission phase (for /dbvf/mission_phase topic)
_STATE_TO_PHASE = {
    MissionState.IDLE: 'IDLE',
    MissionState.PREFLIGHT_CHECK: 'PREFLIGHT',
    MissionState.TAKEOFF_H: 'FM1',
    MissionState.TRANSIT_H_TO_L: 'FM1',
    MissionState.LAND_L: 'FM1',
    MissionState.WAIT_FLAGGER: 'FM1',
    MissionState.TAKEOFF_L: 'FM2',
    MissionState.TRANSIT_TO_DROP: 'FM2',
    MissionState.DROP_PAYLOAD: 'FM2',
    MissionState.TRANSIT_TO_WA: 'FM3',
    MissionState.LAND_WA: 'FM3',
    MissionState.TAKEOFF_WA: 'FM3',
    MissionState.TRANSIT_TO_DROP_2: 'FM3',
    MissionState.DROP_PAYLOAD_2: 'FM3',
    MissionState.TRANSIT_TO_H: 'RTH',
    MissionState.LAND_H: 'RTH',
    MissionState.COMPLETE: 'COMPLETE',
    MissionState.ABORT: 'ABORT',
}


def get_mission_phase(state):
    """Return the mission phase string for a given state."""
    return _STATE_TO_PHASE.get(state, 'UNKNOWN')


class MissionStateMachine:
    def __init__(self, config):
        self.state = MissionState.IDLE
        self.config = config
        self.start_time = None
        self.abort_reason = ''

        # WAIT_FLAGGER resume flag
        self._resume_requested = False

        # DROP_PAYLOAD timing
        self._drop_start_time = None

        # LAND_WA tracking
        self._landing_state = None

        # Guided position resend tracking
        self._last_guided_send_time = 0.0

        # Precompute altitudes in meters
        self._transit_alt_m = ft_to_m(config['transit_altitude_ft'])
        self._takeoff_complete_alt_m = ft_to_m(config['takeoff_complete_alt_ft'])
        self._tolerance = config['position_tolerance_m']

    def start(self):
        """Begin the mission — transition from IDLE to PREFLIGHT_CHECK."""
        self.state = MissionState.PREFLIGHT_CHECK
        self.start_time = None
        self.abort_reason = ''
        self._resume_requested = False
        self._drop_start_time = None
        self._landing_state = None
        self._last_guided_send_time = 0.0

    def resume(self):
        """Signal that the flagger has approved — used during WAIT_FLAGGER."""
        self._resume_requested = True

    def set_landing_state(self, landing_state_str):
        """Called by the ROS node when /dbvf/landing_state updates."""
        self._landing_state = landing_state_str

    def abort(self, reason):
        """Force transition to ABORT from any state."""
        self.abort_reason = reason
        self.state = MissionState.ABORT

    def update(self, vehicle_state, current_time):
        """Advance the state machine. Returns (state, action_dict).

        action_dict keys:
          'action': str describing what happened or is happening
          'reason': str (only on ABORT)
          'entry_actions': list of str describing actions the ROS node should take
        """
        vs = vehicle_state

        if self.state == MissionState.IDLE:
            return self.state, {'action': None, 'entry_actions': []}

        if self.state == MissionState.COMPLETE:
            return self.state, {'action': 'complete', 'entry_actions': []}

        if self.state == MissionState.ABORT:
            return self.state, {'action': 'aborted', 'reason': self.abort_reason,
                                'entry_actions': []}

        # Start mission timer on first non-IDLE update
        if self.start_time is None:
            self.start_time = current_time

        # Global timeout check (skip for WAIT_FLAGGER — no timeout there)
        if (self.state != MissionState.WAIT_FLAGGER
                and current_time - self.start_time > self.config['mission_timeout_s']):
            self.state = MissionState.ABORT
            self.abort_reason = 'Mission timeout'
            return self.state, {'action': 'aborted', 'reason': self.abort_reason,
                                'entry_actions': ['set_mode_land']}

        # Heartbeat check (skip for WAIT_FLAGGER — aircraft is on ground)
        if (self.state != MissionState.WAIT_FLAGGER
                and hasattr(vs, 'heartbeat_ok') and not vs.heartbeat_ok):
            self.state = MissionState.ABORT
            self.abort_reason = 'Heartbeat loss'
            return self.state, {'action': 'aborted', 'reason': self.abort_reason,
                                'entry_actions': ['set_mode_land']}

        # Dispatch to state handler
        handler = self._handlers.get(self.state)
        if handler:
            return handler(self, vs, current_time)
        return self.state, {'action': None, 'entry_actions': []}

    # -- State handlers -------------------------------------------------------

    def _preflight(self, vs, t):
        if not vs.armed:
            self.state = MissionState.ABORT
            self.abort_reason = 'Preflight failed: not armed'
            return self.state, {'action': 'aborted', 'reason': self.abort_reason,
                                'entry_actions': []}
        if vs.mode != 'GUIDED':
            self.state = MissionState.ABORT
            self.abort_reason = 'Preflight failed: mode is not GUIDED'
            return self.state, {'action': 'aborted', 'reason': self.abort_reason,
                                'entry_actions': []}
        self.state = MissionState.TAKEOFF_H
        return self.state, {'action': 'preflight_pass',
                            'entry_actions': ['takeoff']}

    def _takeoff_h(self, vs, t):
        alt = self._get_altitude(vs)
        if alt >= self._takeoff_complete_alt_m:
            self.state = MissionState.TRANSIT_H_TO_L
            return self.state, {'action': 'takeoff_complete',
                                'entry_actions': ['send_guided_position_l']}
        return self.state, {'action': 'climbing', 'entry_actions': []}

    def _transit_h_to_l(self, vs, t):
        target_lat = self.config['landing_lat']
        target_lon = self.config['landing_lon']
        dist = haversine_distance_m(vs.lat, vs.lon, target_lat, target_lon)
        if dist < self._tolerance:
            self.state = MissionState.LAND_L
            return self.state, {'action': 'arrived_l',
                                'entry_actions': ['set_mode_land']}
        entry = []
        if t - self._last_guided_send_time >= self.config['guided_resend_interval_s']:
            entry.append('send_guided_position_l')
            self._last_guided_send_time = t
        return self.state, {'action': 'transiting', 'entry_actions': entry}

    def _land_l(self, vs, t):
        if self._is_landed(vs):
            self.state = MissionState.WAIT_FLAGGER
            return self.state, {'action': 'landed_l', 'entry_actions': []}
        return self.state, {'action': 'landing', 'entry_actions': []}

    def _wait_flagger(self, vs, t):
        if self._resume_requested:
            self._resume_requested = False
            self.state = MissionState.TAKEOFF_L
            return self.state, {'action': 'flagger_resume',
                                'entry_actions': ['arm', 'set_mode_guided',
                                                  'takeoff']}
        return self.state, {'action': 'waiting', 'entry_actions': []}

    def _takeoff_l(self, vs, t):
        alt = self._get_altitude(vs)
        if alt >= self._takeoff_complete_alt_m:
            self.state = MissionState.TRANSIT_TO_DROP
            return self.state, {'action': 'takeoff_complete',
                                'entry_actions': ['send_guided_position_drop']}
        return self.state, {'action': 'climbing', 'entry_actions': []}

    def _transit_to_drop(self, vs, t):
        drop_lat, drop_lon = self._get_drop_coords()
        dist = haversine_distance_m(vs.lat, vs.lon, drop_lat, drop_lon)
        if dist < self._tolerance:
            self._drop_start_time = t
            self.state = MissionState.DROP_PAYLOAD
            return self.state, {'action': 'arrived_drop',
                                'entry_actions': ['servo_release']}
        entry = []
        if t - self._last_guided_send_time >= self.config['guided_resend_interval_s']:
            entry.append('send_guided_position_drop')
            self._last_guided_send_time = t
        return self.state, {'action': 'transiting', 'entry_actions': entry}

    def _drop_payload(self, vs, t):
        elapsed = t - self._drop_start_time
        if elapsed >= self.config['drop_settle_time_s']:
            self.state = MissionState.TRANSIT_TO_WA
            self._last_guided_send_time = 0.0
            return self.state, {'action': 'drop_complete',
                                'entry_actions': ['send_guided_position_wa']}
        return self.state, {'action': 'dropping', 'entry_actions': []}

    def _transit_to_wa(self, vs, t):
        wa_lat = self.config['wa_lat']
        wa_lon = self.config['wa_lon']
        dist = haversine_distance_m(vs.lat, vs.lon, wa_lat, wa_lon)
        if dist < self._tolerance:
            self.state = MissionState.LAND_WA
            self._landing_state = None
            return self.state, {'action': 'arrived_wa',
                                'entry_actions': ['start_precision_landing']}
        entry = []
        if t - self._last_guided_send_time >= self.config['guided_resend_interval_s']:
            entry.append('send_guided_position_wa')
            self._last_guided_send_time = t
        return self.state, {'action': 'transiting', 'entry_actions': entry}

    def _land_wa(self, vs, t):
        if self._landing_state == 'LANDED':
            self.state = MissionState.TAKEOFF_WA
            return self.state, {'action': 'landed_wa',
                                'entry_actions': ['arm', 'set_mode_guided',
                                                  'takeoff']}
        if self._landing_state == 'ABORT_LAND':
            self.state = MissionState.ABORT
            self.abort_reason = 'Precision landing failed at WA'
            return self.state, {'action': 'aborted', 'reason': self.abort_reason,
                                'entry_actions': ['set_mode_land']}
        return self.state, {'action': 'precision_landing', 'entry_actions': []}

    def _takeoff_wa(self, vs, t):
        alt = self._get_altitude(vs)
        if alt >= self._takeoff_complete_alt_m:
            self.state = MissionState.TRANSIT_TO_DROP_2
            self._last_guided_send_time = 0.0
            return self.state, {'action': 'takeoff_complete',
                                'entry_actions': ['send_guided_position_drop']}
        return self.state, {'action': 'climbing', 'entry_actions': []}

    def _transit_to_drop_2(self, vs, t):
        drop_lat, drop_lon = self._get_drop_coords()
        dist = haversine_distance_m(vs.lat, vs.lon, drop_lat, drop_lon)
        if dist < self._tolerance:
            self._drop_start_time = t
            self.state = MissionState.DROP_PAYLOAD_2
            return self.state, {'action': 'arrived_drop_2',
                                'entry_actions': ['servo_release']}
        entry = []
        if t - self._last_guided_send_time >= self.config['guided_resend_interval_s']:
            entry.append('send_guided_position_drop')
            self._last_guided_send_time = t
        return self.state, {'action': 'transiting', 'entry_actions': entry}

    def _drop_payload_2(self, vs, t):
        elapsed = t - self._drop_start_time
        if elapsed >= self.config['drop_settle_time_s']:
            self.state = MissionState.TRANSIT_TO_H
            self._last_guided_send_time = 0.0
            return self.state, {'action': 'drop_complete',
                                'entry_actions': ['send_guided_position_h']}
        return self.state, {'action': 'dropping', 'entry_actions': []}

    def _transit_to_h(self, vs, t):
        h_lat = self.config['home_lat']
        h_lon = self.config['home_lon']
        dist = haversine_distance_m(vs.lat, vs.lon, h_lat, h_lon)
        if dist < self._tolerance:
            self.state = MissionState.LAND_H
            return self.state, {'action': 'arrived_h',
                                'entry_actions': ['set_mode_land']}
        entry = []
        if t - self._last_guided_send_time >= self.config['guided_resend_interval_s']:
            entry.append('send_guided_position_h')
            self._last_guided_send_time = t
        return self.state, {'action': 'transiting', 'entry_actions': entry}

    def _land_h(self, vs, t):
        if self._is_landed(vs):
            self.state = MissionState.COMPLETE
            return self.state, {'action': 'landed_h', 'entry_actions': []}
        return self.state, {'action': 'landing', 'entry_actions': []}

    # Handler dispatch table (defined after methods so they exist)
    _handlers = {
        MissionState.PREFLIGHT_CHECK: _preflight,
        MissionState.TAKEOFF_H: _takeoff_h,
        MissionState.TRANSIT_H_TO_L: _transit_h_to_l,
        MissionState.LAND_L: _land_l,
        MissionState.WAIT_FLAGGER: _wait_flagger,
        MissionState.TAKEOFF_L: _takeoff_l,
        MissionState.TRANSIT_TO_DROP: _transit_to_drop,
        MissionState.DROP_PAYLOAD: _drop_payload,
        MissionState.TRANSIT_TO_WA: _transit_to_wa,
        MissionState.LAND_WA: _land_wa,
        MissionState.TAKEOFF_WA: _takeoff_wa,
        MissionState.TRANSIT_TO_DROP_2: _transit_to_drop_2,
        MissionState.DROP_PAYLOAD_2: _drop_payload_2,
        MissionState.TRANSIT_TO_H: _transit_to_h,
        MissionState.LAND_H: _land_h,
    }

    # -- Helpers --------------------------------------------------------------

    def _get_altitude(self, vs):
        """Return best available altitude in meters."""
        if self.config.get('prefer_rangefinder', True) and vs.range_alt >= 0.0:
            return vs.range_alt
        return vs.alt_rel

    def _get_drop_coords(self):
        """Return (lat, lon) of the configured drop target."""
        if self.config['drop_target'] == 'F2':
            return self.config['f2_lat'], self.config['f2_lon']
        return self.config['f1_lat'], self.config['f1_lon']

    @staticmethod
    def _is_landed(vs):
        """Detect landed condition: disarmed OR (low alt AND low vertical speed)."""
        if not vs.armed:
            return True
        return vs.alt_rel < 0.3 and abs(vs.vz) < 0.1
