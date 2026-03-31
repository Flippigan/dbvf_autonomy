"""Tests for mission state machine — pure Python, no ROS2."""
from dbvf_autonomy.mission_state_machine import MissionStateMachine, MissionState
from dbvf_autonomy.mission_helpers import DEFAULT_MISSION_CONFIG, ft_to_m


# ---------------------------------------------------------------------------
# Mock vehicle state (same pattern as test_state_machine.py)
# ---------------------------------------------------------------------------

class MockVehicleState:
    def __init__(self, lat=-35.3632621, lon=149.1652374, alt_rel=0.0,
                 armed=True, mode='GUIDED', vz=0.0, range_alt=-1.0,
                 heartbeat_ok=True):
        self.lat = lat
        self.lon = lon
        self.alt_rel = alt_rel
        self.armed = armed
        self.mode = mode
        self.vz = vz
        self.range_alt = range_alt
        self.heartbeat_ok = heartbeat_ok


def _make_config(**overrides):
    cfg = dict(DEFAULT_MISSION_CONFIG)
    cfg.update(overrides)
    return cfg


# Convenient GPS coords from default config
H_LAT, H_LON = -35.3632621, 149.1652374
L_LAT, L_LON = -35.3640000, 149.1652374
WA_LAT, WA_LON = -35.3632531, 149.1657896
F1_LAT, F1_LON = -35.3650000, 149.1652374


# ---------------------------------------------------------------------------
# Basic state + start
# ---------------------------------------------------------------------------

def test_starts_idle():
    sm = MissionStateMachine(_make_config())
    assert sm.state == MissionState.IDLE


def test_start_transitions_to_preflight():
    sm = MissionStateMachine(_make_config())
    sm.start()
    assert sm.state == MissionState.PREFLIGHT_CHECK


# ---------------------------------------------------------------------------
# PREFLIGHT_CHECK
# ---------------------------------------------------------------------------

def test_preflight_pass():
    sm = MissionStateMachine(_make_config())
    sm.start()
    vs = MockVehicleState(armed=True, mode='GUIDED')
    state, info = sm.update(vs, 0.0)
    assert state == MissionState.TAKEOFF_H
    assert info['action'] == 'preflight_pass'


def test_preflight_fail_not_armed():
    sm = MissionStateMachine(_make_config())
    sm.start()
    vs = MockVehicleState(armed=False, mode='GUIDED')
    state, info = sm.update(vs, 0.0)
    assert state == MissionState.ABORT
    assert 'armed' in info['reason'].lower()


def test_preflight_fail_wrong_mode():
    sm = MissionStateMachine(_make_config())
    sm.start()
    vs = MockVehicleState(armed=True, mode='STABILIZE')
    state, info = sm.update(vs, 0.0)
    assert state == MissionState.ABORT
    assert 'mode' in info['reason'].lower()


# ---------------------------------------------------------------------------
# TAKEOFF_H
# ---------------------------------------------------------------------------

def _to_takeoff_h(sm):
    sm.start()
    vs = MockVehicleState(armed=True, mode='GUIDED')
    sm.update(vs, 0.0)
    assert sm.state == MissionState.TAKEOFF_H
    return vs


def test_takeoff_h_waits_for_altitude():
    sm = MissionStateMachine(_make_config())
    _to_takeoff_h(sm)
    vs = MockVehicleState(alt_rel=5.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 1.0)
    assert state == MissionState.TAKEOFF_H
    assert info['action'] == 'climbing'


def test_takeoff_h_complete_with_rangefinder():
    sm = MissionStateMachine(_make_config(prefer_rangefinder=True))
    _to_takeoff_h(sm)
    alt = ft_to_m(33.0) + 0.1  # Just above 33ft
    vs = MockVehicleState(alt_rel=alt, range_alt=alt, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 1.0)
    assert state == MissionState.TRANSIT_H_TO_L
    assert info['action'] == 'takeoff_complete'


def test_takeoff_h_complete_fallback_to_alt_rel():
    sm = MissionStateMachine(_make_config(prefer_rangefinder=True))
    _to_takeoff_h(sm)
    alt = ft_to_m(33.0) + 0.1
    vs = MockVehicleState(alt_rel=alt, range_alt=-1.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 1.0)
    assert state == MissionState.TRANSIT_H_TO_L


# ---------------------------------------------------------------------------
# TRANSIT_H_TO_L
# ---------------------------------------------------------------------------

def _to_transit_h_to_l(sm):
    _to_takeoff_h(sm)
    alt = ft_to_m(33.0) + 0.1
    vs = MockVehicleState(alt_rel=alt, range_alt=alt, armed=True, mode='GUIDED')
    sm.update(vs, 1.0)
    assert sm.state == MissionState.TRANSIT_H_TO_L
    return vs


def test_transit_h_to_l_not_arrived():
    sm = MissionStateMachine(_make_config())
    _to_transit_h_to_l(sm)
    vs = MockVehicleState(lat=H_LAT, lon=H_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 2.0)
    assert state == MissionState.TRANSIT_H_TO_L
    assert info['action'] == 'transiting'


def test_transit_h_to_l_arrived():
    sm = MissionStateMachine(_make_config())
    _to_transit_h_to_l(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 2.0)
    assert state == MissionState.LAND_L
    assert info['action'] == 'arrived_l'


# ---------------------------------------------------------------------------
# LAND_L
# ---------------------------------------------------------------------------

def _to_land_l(sm):
    _to_transit_h_to_l(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    sm.update(vs, 2.0)
    assert sm.state == MissionState.LAND_L


def test_land_l_waits_while_airborne():
    sm = MissionStateMachine(_make_config())
    _to_land_l(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=5.0, armed=True, vz=-0.5)
    state, info = sm.update(vs, 3.0)
    assert state == MissionState.LAND_L
    assert info['action'] == 'landing'


def test_land_l_complete_disarmed():
    sm = MissionStateMachine(_make_config())
    _to_land_l(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=0.1, armed=False, vz=0.0)
    state, info = sm.update(vs, 3.0)
    assert state == MissionState.WAIT_FLAGGER
    assert info['action'] == 'landed_l'


def test_land_l_complete_low_and_slow():
    sm = MissionStateMachine(_make_config())
    _to_land_l(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=0.2, armed=True, vz=0.05)
    state, info = sm.update(vs, 3.0)
    assert state == MissionState.WAIT_FLAGGER


# ---------------------------------------------------------------------------
# WAIT_FLAGGER
# ---------------------------------------------------------------------------

def _to_wait_flagger(sm):
    _to_land_l(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=0.1, armed=False)
    sm.update(vs, 3.0)
    assert sm.state == MissionState.WAIT_FLAGGER


def test_wait_flagger_stays_without_resume():
    sm = MissionStateMachine(_make_config())
    _to_wait_flagger(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=0.0, armed=False)
    state, info = sm.update(vs, 100.0)
    assert state == MissionState.WAIT_FLAGGER
    assert info['action'] == 'waiting'


def test_wait_flagger_resumes():
    sm = MissionStateMachine(_make_config())
    _to_wait_flagger(sm)
    sm.resume()
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=0.0, armed=False)
    state, info = sm.update(vs, 4.0)
    assert state == MissionState.TAKEOFF_L
    assert info['action'] == 'flagger_resume'


# ---------------------------------------------------------------------------
# FM-2: TAKEOFF_L -> TRANSIT_TO_DROP -> DROP_PAYLOAD
# ---------------------------------------------------------------------------

def _to_takeoff_l(sm):
    _to_wait_flagger(sm)
    sm.resume()
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=0.0, armed=False)
    sm.update(vs, 4.0)
    assert sm.state == MissionState.TAKEOFF_L
    return vs


def test_takeoff_l_waits_for_altitude():
    sm = MissionStateMachine(_make_config())
    _to_takeoff_l(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=5.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 5.0)
    assert state == MissionState.TAKEOFF_L
    assert info['action'] == 'climbing'


def test_takeoff_l_complete():
    sm = MissionStateMachine(_make_config())
    _to_takeoff_l(sm)
    alt = ft_to_m(33.0) + 0.1
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=alt, range_alt=alt,
                          armed=True, mode='GUIDED')
    state, info = sm.update(vs, 5.0)
    assert state == MissionState.TRANSIT_TO_DROP


def _to_transit_to_drop(sm):
    _to_takeoff_l(sm)
    alt = ft_to_m(33.0) + 0.1
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=alt, range_alt=alt,
                          armed=True, mode='GUIDED')
    sm.update(vs, 5.0)
    assert sm.state == MissionState.TRANSIT_TO_DROP


def test_transit_to_drop_not_arrived():
    sm = MissionStateMachine(_make_config())
    _to_transit_to_drop(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 6.0)
    assert state == MissionState.TRANSIT_TO_DROP
    assert info['action'] == 'transiting'


def test_transit_to_drop_arrived():
    sm = MissionStateMachine(_make_config())
    _to_transit_to_drop(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 6.0)
    assert state == MissionState.DROP_PAYLOAD
    assert info['action'] == 'arrived_drop'
    assert 'servo_release' in info['entry_actions']


def _to_drop_payload(sm):
    _to_transit_to_drop(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    sm.update(vs, 6.0)
    assert sm.state == MissionState.DROP_PAYLOAD


def test_drop_payload_waits_settle():
    sm = MissionStateMachine(_make_config())
    _to_drop_payload(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 7.0)  # 1.0s < 2.0s settle
    assert state == MissionState.DROP_PAYLOAD
    assert info['action'] == 'dropping'


def test_drop_payload_complete():
    sm = MissionStateMachine(_make_config())
    _to_drop_payload(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 8.1)  # 2.1s > 2.0s settle
    assert state == MissionState.TRANSIT_TO_WA
    assert info['action'] == 'drop_complete'


# ---------------------------------------------------------------------------
# FM-3: TRANSIT_TO_WA -> LAND_WA_DESCEND -> WA_DROP_OLD_PAYLOAD ->
#        WA_SERVO_RESET -> LAND_WA_FINAL -> TAKEOFF_WA -> ...
# ---------------------------------------------------------------------------

def _to_transit_to_wa(sm):
    _to_drop_payload(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    sm.update(vs, 8.1)  # drop_settle_time elapsed
    assert sm.state == MissionState.TRANSIT_TO_WA


def test_transit_to_wa_not_arrived():
    sm = MissionStateMachine(_make_config())
    _to_transit_to_wa(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 9.0)
    assert state == MissionState.TRANSIT_TO_WA


def test_transit_to_wa_arrived():
    sm = MissionStateMachine(_make_config())
    _to_transit_to_wa(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 9.0)
    assert state == MissionState.LAND_WA_DESCEND
    assert 'start_precision_landing' in info['entry_actions']


# -- LAND_WA_DESCEND --------------------------------------------------------

def _to_land_wa_descend(sm):
    _to_transit_to_wa(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    sm.update(vs, 9.0)
    assert sm.state == MissionState.LAND_WA_DESCEND


def test_land_wa_descend_waiting():
    sm = MissionStateMachine(_make_config())
    _to_land_wa_descend(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=5.0, armed=True, mode='LAND')
    state, info = sm.update(vs, 10.0)
    assert state == MissionState.LAND_WA_DESCEND
    assert info['action'] == 'precision_landing'


def test_land_wa_descend_to_drop():
    sm = MissionStateMachine(_make_config())
    _to_land_wa_descend(sm)
    sm.set_landing_state('DESCEND_HOLD')
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    state, info = sm.update(vs, 10.0)
    assert state == MissionState.WA_DROP_OLD_PAYLOAD
    assert info['action'] == 'descend_hold_reached'
    assert 'arduino_servo_release' in info['entry_actions']


def test_land_wa_descend_abort():
    sm = MissionStateMachine(_make_config())
    _to_land_wa_descend(sm)
    sm.set_landing_state('ABORT_LAND')
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=1.0, armed=True)
    state, info = sm.update(vs, 10.0)
    assert state == MissionState.ABORT
    assert 'Precision landing failed' in info['reason']


# -- WA_DROP_OLD_PAYLOAD -----------------------------------------------------

def _to_wa_drop_old_payload(sm):
    _to_land_wa_descend(sm)
    sm.set_landing_state('DESCEND_HOLD')
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    sm.update(vs, 10.0)
    assert sm.state == MissionState.WA_DROP_OLD_PAYLOAD


def test_wa_drop_old_payload_waiting():
    sm = MissionStateMachine(_make_config())
    _to_wa_drop_old_payload(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    state, info = sm.update(vs, 11.0)  # 1.0s < 2.0s settle
    assert state == MissionState.WA_DROP_OLD_PAYLOAD
    assert info['action'] == 'dropping_old_payload'


def test_wa_drop_old_payload_complete():
    sm = MissionStateMachine(_make_config())
    _to_wa_drop_old_payload(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    state, info = sm.update(vs, 12.1)  # 2.1s > 2.0s settle
    assert state == MissionState.WA_SERVO_RESET
    assert info['action'] == 'drop_old_complete'
    assert 'arduino_servo_pickup' in info['entry_actions']


# -- WA_SERVO_RESET ----------------------------------------------------------

def _to_wa_servo_reset(sm):
    _to_wa_drop_old_payload(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    sm.update(vs, 12.1)  # settle elapsed
    assert sm.state == MissionState.WA_SERVO_RESET


def test_wa_servo_reset_waiting():
    sm = MissionStateMachine(_make_config())
    _to_wa_servo_reset(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    state, info = sm.update(vs, 13.0)  # 0.9s < 2.0s settle
    assert state == MissionState.WA_SERVO_RESET
    assert info['action'] == 'resetting_servo'


def test_wa_servo_reset_complete():
    sm = MissionStateMachine(_make_config())
    _to_wa_servo_reset(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    state, info = sm.update(vs, 14.2)  # 2.1s > 2.0s settle
    assert state == MissionState.LAND_WA_FINAL
    assert info['action'] == 'servo_reset_complete'


# -- LAND_WA_FINAL -----------------------------------------------------------

def _to_land_wa_final(sm):
    _to_wa_servo_reset(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    sm.update(vs, 14.2)  # settle elapsed
    assert sm.state == MissionState.LAND_WA_FINAL


def test_land_wa_final_waiting():
    sm = MissionStateMachine(_make_config())
    _to_land_wa_final(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.3, armed=True)
    state, info = sm.update(vs, 15.0)
    assert state == MissionState.LAND_WA_FINAL
    assert info['action'] == 'final_descent'


def test_land_wa_final_success():
    sm = MissionStateMachine(_make_config())
    _to_land_wa_final(sm)
    sm.set_landing_state('LANDED')
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.1, armed=False)
    state, info = sm.update(vs, 15.0)
    assert state == MissionState.TAKEOFF_WA
    assert info['action'] == 'landed_wa'


def test_land_wa_final_abort():
    sm = MissionStateMachine(_make_config())
    _to_land_wa_final(sm)
    sm.set_landing_state('ABORT_LAND')
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    state, info = sm.update(vs, 15.0)
    assert state == MissionState.ABORT
    assert 'Precision landing failed' in info['reason']


# -- TAKEOFF_WA (updated helper) ---------------------------------------------

def _to_takeoff_wa(sm):
    _to_land_wa_final(sm)
    sm.set_landing_state('LANDED')
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.1, armed=False)
    sm.update(vs, 15.0)
    assert sm.state == MissionState.TAKEOFF_WA


def test_takeoff_wa_complete():
    sm = MissionStateMachine(_make_config())
    _to_takeoff_wa(sm)
    alt = ft_to_m(33.0) + 0.1
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=alt, range_alt=alt,
                          armed=True, mode='GUIDED')
    state, info = sm.update(vs, 11.0)
    assert state == MissionState.TRANSIT_TO_DROP_2


def _to_transit_to_drop_2(sm):
    _to_takeoff_wa(sm)
    alt = ft_to_m(33.0) + 0.1
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=alt, range_alt=alt,
                          armed=True, mode='GUIDED')
    sm.update(vs, 11.0)
    assert sm.state == MissionState.TRANSIT_TO_DROP_2


def test_transit_to_drop_2_arrived():
    sm = MissionStateMachine(_make_config())
    _to_transit_to_drop_2(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 12.0)
    assert state == MissionState.DROP_PAYLOAD_2
    assert 'servo_release' in info['entry_actions']


def _to_drop_payload_2(sm):
    _to_transit_to_drop_2(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    sm.update(vs, 12.0)
    assert sm.state == MissionState.DROP_PAYLOAD_2


def test_drop_payload_2_complete():
    sm = MissionStateMachine(_make_config())
    _to_drop_payload_2(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 14.1)  # 2.1s > 2.0s settle
    assert state == MissionState.TRANSIT_TO_H
    assert info['action'] == 'drop_complete'


# ---------------------------------------------------------------------------
# Return Home: TRANSIT_TO_H -> LAND_H -> COMPLETE
# ---------------------------------------------------------------------------

def _to_transit_to_h(sm):
    _to_drop_payload_2(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    sm.update(vs, 14.1)
    assert sm.state == MissionState.TRANSIT_TO_H


def test_transit_to_h_arrived():
    sm = MissionStateMachine(_make_config())
    _to_transit_to_h(sm)
    vs = MockVehicleState(lat=H_LAT, lon=H_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 15.0)
    assert state == MissionState.LAND_H
    assert 'set_mode_land' in info['entry_actions']


def _to_land_h(sm):
    _to_transit_to_h(sm)
    vs = MockVehicleState(lat=H_LAT, lon=H_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    sm.update(vs, 15.0)
    assert sm.state == MissionState.LAND_H


def test_land_h_complete():
    sm = MissionStateMachine(_make_config())
    _to_land_h(sm)
    vs = MockVehicleState(lat=H_LAT, lon=H_LON, alt_rel=0.1, armed=False, vz=0.0)
    state, info = sm.update(vs, 16.0)
    assert state == MissionState.COMPLETE
    assert info['action'] == 'landed_h'


# ---------------------------------------------------------------------------
# ABORT: reachable from any state
# ---------------------------------------------------------------------------

def test_abort_manual():
    sm = MissionStateMachine(_make_config())
    _to_transit_h_to_l(sm)
    sm.abort('Manual abort')
    vs = MockVehicleState()
    state, info = sm.update(vs, 5.0)
    assert state == MissionState.ABORT
    assert info['reason'] == 'Manual abort'


def test_abort_timeout():
    sm = MissionStateMachine(_make_config(mission_timeout_s=10.0))
    _to_takeoff_h(sm)
    vs = MockVehicleState(alt_rel=5.0, armed=True, mode='GUIDED')
    sm.update(vs, 0.0)  # set start_time
    state, info = sm.update(vs, 11.0)  # 11s > 10s timeout
    assert state == MissionState.ABORT
    assert 'timeout' in info['reason'].lower()


def test_abort_heartbeat_loss():
    sm = MissionStateMachine(_make_config())
    _to_transit_h_to_l(sm)
    vs = MockVehicleState(lat=H_LAT, lon=H_LON, alt_rel=11.0, armed=True,
                          mode='GUIDED', heartbeat_ok=False)
    state, info = sm.update(vs, 5.0)
    assert state == MissionState.ABORT
    assert 'heartbeat' in info['reason'].lower()


def test_wait_flagger_ignores_timeout():
    """WAIT_FLAGGER should NOT abort on mission timeout."""
    sm = MissionStateMachine(_make_config(mission_timeout_s=10.0))
    _to_wait_flagger(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=0.0, armed=False)
    state, info = sm.update(vs, 1000.0)  # Way past timeout
    assert state == MissionState.WAIT_FLAGGER


# ---------------------------------------------------------------------------
# Drop target selection
# ---------------------------------------------------------------------------

def test_drop_target_f2():
    sm = MissionStateMachine(_make_config(drop_target='F2'))
    lat, lon = sm._get_drop_coords()
    assert lat == -35.3660000
    assert lon == 149.1652374


# ---------------------------------------------------------------------------
# Mission phase mapping
# ---------------------------------------------------------------------------

def test_phase_fm1():
    from dbvf_autonomy.mission_state_machine import get_mission_phase
    assert get_mission_phase(MissionState.TAKEOFF_H) == 'FM1'
    assert get_mission_phase(MissionState.TRANSIT_H_TO_L) == 'FM1'
    assert get_mission_phase(MissionState.LAND_L) == 'FM1'


def test_phase_fm2():
    from dbvf_autonomy.mission_state_machine import get_mission_phase
    assert get_mission_phase(MissionState.TAKEOFF_L) == 'FM2'
    assert get_mission_phase(MissionState.DROP_PAYLOAD) == 'FM2'


def test_phase_fm3():
    from dbvf_autonomy.mission_state_machine import get_mission_phase
    assert get_mission_phase(MissionState.LAND_WA_DESCEND) == 'FM3'
    assert get_mission_phase(MissionState.WA_DROP_OLD_PAYLOAD) == 'FM3'
    assert get_mission_phase(MissionState.WA_SERVO_RESET) == 'FM3'
    assert get_mission_phase(MissionState.LAND_WA_FINAL) == 'FM3'
    assert get_mission_phase(MissionState.DROP_PAYLOAD_2) == 'FM3'


def test_phase_rth():
    from dbvf_autonomy.mission_state_machine import get_mission_phase
    assert get_mission_phase(MissionState.TRANSIT_TO_H) == 'RTH'
    assert get_mission_phase(MissionState.LAND_H) == 'RTH'
