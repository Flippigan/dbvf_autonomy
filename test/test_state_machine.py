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
