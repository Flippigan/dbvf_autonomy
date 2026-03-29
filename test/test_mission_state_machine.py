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
