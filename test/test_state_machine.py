from dbvf_autonomy.precision_landing_node import LandingStateMachine, LandingState, PIDController


class MockVehicleState:
    def __init__(self, lat=0.0, lon=0.0, alt_rel=10.0, armed=True, vz=0.0):
        self.lat = lat
        self.lon = lon
        self.alt_rel = alt_rel
        self.armed = armed
        self.vz = vz


class MockTagStatus:
    def __init__(self, detected=False, active_tag_id=-1):
        self.detected = detected
        self.active_tag_id = active_tag_id


CONFIG = {
    'approach_altitude': 8.0,
    'min_search_altitude': 1.0,
    'search_descent_rate': 0.3,
    'position_tolerance': 2.0,
    'tag_confirm_frames': 5,
    'tag_lost_timeout': 4.0,
    'landing_timeout': 60.0,
    'descend_floor_altitude': 1.5,
    'final_descent_rate': 0.15,
    'small_tag_confirm_time': 2.0,
    'hold_stabilize_time': 1.0,
    'offset_forward': 0.0,
    'offset_right': 0.0,
    'offset_tolerance': 0.05,
    'small_tag_search_radius': 0.5,
    'small_tag_search_speed': 0.2,
    'secondary_tag_id': 1,
}

# Target coordinates for all tests
LAT = -35.363262
LON = 149.165237


# ---------------------------------------------------------------------------
# Helper to advance FSM to a given state
# ---------------------------------------------------------------------------

def _to_search(sm):
    """Advance from IDLE to SEARCH."""
    sm.start(LAT, LON)
    vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=8.0)
    sm.update(vs, None, 1.0)  # -> SEARCH
    assert sm.state == LandingState.SEARCH
    return vs


def _to_descend_coarse(sm):
    """Advance from IDLE to DESCEND_COARSE."""
    vs = _to_search(sm)
    tag = MockTagStatus(detected=True, active_tag_id=0)
    for i in range(5):
        sm.update(vs, tag, 2.0 + i * 0.05)
    assert sm.state == LandingState.DESCEND_COARSE
    return vs


# ---------------------------------------------------------------------------
# Original tests (updated for new action dict format)
# ---------------------------------------------------------------------------

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
    state, info = sm.update(vs, None, 1.0)
    assert state == LandingState.SEARCH
    assert info['action'] == 'approach_complete'


def test_approach_stays_if_too_far():
    sm = LandingStateMachine(CONFIG)
    sm.start(LAT, LON)
    # ~111m away in latitude
    vs = MockVehicleState(lat=LAT + 0.001, lon=LON, alt_rel=8.0)
    state, info = sm.update(vs, None, 1.0)
    assert state == LandingState.APPROACH
    assert info['action'] == 'approaching'


def test_approach_stays_if_too_high():
    sm = LandingStateMachine(CONFIG)
    sm.start(LAT, LON)
    vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=20.0)
    state, info = sm.update(vs, None, 1.0)
    assert state == LandingState.APPROACH


def test_search_to_descend_coarse_with_confirmation():
    sm = LandingStateMachine(CONFIG)
    vs = _to_search(sm)

    tag = MockTagStatus(detected=True, active_tag_id=0)
    for i in range(4):
        state, _ = sm.update(vs, tag, 2.0 + i * 0.05)
        assert state == LandingState.SEARCH  # Not confirmed yet

    state, info = sm.update(vs, tag, 2.25)
    assert state == LandingState.DESCEND_COARSE
    assert info['action'] == 'tag_confirmed'


def test_search_resets_confirm_on_loss():
    sm = LandingStateMachine(CONFIG)
    vs = _to_search(sm)

    tag = MockTagStatus(detected=True, active_tag_id=0)
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
    _to_search(sm)

    low_vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=0.5)
    no_tag = MockTagStatus(detected=False)
    state, info = sm.update(low_vs, no_tag, 2.0)
    assert state == LandingState.ABORT_LAND
    assert info['action'] == 'below_min_alt'


def test_descend_coarse_to_landed_on_disarm():
    sm = LandingStateMachine(CONFIG)
    _to_descend_coarse(sm)

    landed = MockVehicleState(lat=LAT, lon=LON, alt_rel=0.05, armed=False, vz=0.0)
    tag = MockTagStatus(detected=True, active_tag_id=0)
    state, info = sm.update(landed, tag, 10.0)
    assert state == LandingState.LANDED
    assert info['action'] == 'landed'


def test_descend_coarse_to_search_on_tag_lost_timeout():
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    no_tag = MockTagStatus(detected=False)
    state, _ = sm.update(vs, no_tag, 3.0)  # Start lost timer
    assert state == LandingState.DESCEND_COARSE

    state, _ = sm.update(vs, no_tag, 5.0)  # 2s elapsed < 4s timeout
    assert state == LandingState.DESCEND_COARSE

    state, info = sm.update(vs, no_tag, 7.1)  # 4.1s > 4s timeout
    assert state == LandingState.SEARCH
    assert info['action'] == 'tag_lost'


def test_descend_coarse_tag_reacquired_resets_lost_timer():
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    no_tag = MockTagStatus(detected=False)
    tag = MockTagStatus(detected=True, active_tag_id=0)
    sm.update(vs, no_tag, 3.0)   # Lost at t=3.0
    sm.update(vs, no_tag, 5.0)   # Still lost at t=5.0 (2s < 4s)
    sm.update(vs, tag, 5.5)      # Reacquired — resets timer
    sm.update(vs, no_tag, 6.0)   # Lost again at t=6.0
    state, _ = sm.update(vs, no_tag, 9.5)  # 3.5s since re-loss < 4s
    assert state == LandingState.DESCEND_COARSE


def test_abort_to_landed():
    sm = LandingStateMachine(CONFIG)
    _to_search(sm)

    low_vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=0.5)
    sm.update(low_vs, MockTagStatus(detected=False), 2.0)  # -> ABORT_LAND
    assert sm.state == LandingState.ABORT_LAND

    landed = MockVehicleState(lat=LAT, lon=LON, alt_rel=0.05, armed=False, vz=0.0)
    state, info = sm.update(landed, None, 10.0)
    assert state == LandingState.LANDED


def test_global_timeout():
    sm = LandingStateMachine(CONFIG)
    sm.start(LAT, LON)
    vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=20.0)
    sm.update(vs, None, 0.0)  # start_time=0
    state, info = sm.update(vs, None, 61.0)  # 61 > 60s timeout
    assert state == LandingState.ABORT_LAND
    assert info['action'] == 'timeout'


# ---------------------------------------------------------------------------
# New transition tests for expanded FSM
# ---------------------------------------------------------------------------

def test_descend_coarse_to_hold():
    """DESCEND_COARSE -> DESCEND_HOLD when small tag detected for small_tag_confirm_time."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    small_tag = MockTagStatus(detected=True, active_tag_id=1)
    # First detection starts the timer
    sm.update(vs, small_tag, 10.0)
    assert sm.state == LandingState.DESCEND_COARSE

    # Not enough time yet
    sm.update(vs, small_tag, 11.0)
    assert sm.state == LandingState.DESCEND_COARSE

    # 2.0 seconds of continuous detection → transition
    state, info = sm.update(vs, small_tag, 12.1)
    assert state == LandingState.DESCEND_HOLD
    assert info['action'] == 'small_tag_confirmed'


def test_descend_coarse_to_search_pattern():
    """DESCEND_COARSE -> SMALL_TAG_SEARCH at descend_floor_altitude, no small tag."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    low_vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=1.5)
    large_tag = MockTagStatus(detected=True, active_tag_id=0)
    state, info = sm.update(low_vs, large_tag, 10.0)
    assert state == LandingState.SMALL_TAG_SEARCH
    assert info['action'] == 'floor_altitude'


def test_descend_coarse_tag_lost():
    """DESCEND_COARSE -> SEARCH when tags lost for tag_lost_timeout."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    no_tag = MockTagStatus(detected=False)
    sm.update(vs, no_tag, 10.0)  # Start lost timer
    sm.update(vs, no_tag, 12.0)  # 2s < 4s
    state, info = sm.update(vs, no_tag, 14.1)  # 4.1s > 4s
    assert state == LandingState.SEARCH
    assert info['action'] == 'tag_lost'


def test_descend_hold_to_offset():
    """DESCEND_HOLD -> DESCEND_OFFSET after hold_stabilize_time."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    small_tag = MockTagStatus(detected=True, active_tag_id=1)
    # Get to DESCEND_HOLD
    sm.update(vs, small_tag, 10.0)
    sm.update(vs, small_tag, 12.1)  # -> DESCEND_HOLD
    assert sm.state == LandingState.DESCEND_HOLD

    # Wait hold_stabilize_time (1.0s)
    sm.update(vs, small_tag, 12.5)  # 0.4s < 1.0s
    assert sm.state == LandingState.DESCEND_HOLD

    state, info = sm.update(vs, small_tag, 13.2)  # 1.1s > 1.0s
    assert state == LandingState.DESCEND_OFFSET
    assert info['action'] == 'hold_complete'


def test_descend_offset_to_final():
    """DESCEND_OFFSET -> DESCEND_FINAL when position error < offset_tolerance.

    Note: The actual offset checking is done in the ROS node's _velocity_servo,
    which directly sets fsm.state. The FSM's _descend_offset returns use_offset=True
    to signal the node to check. We test the FSM returns the correct info.
    """
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    small_tag = MockTagStatus(detected=True, active_tag_id=1)
    sm.update(vs, small_tag, 10.0)
    sm.update(vs, small_tag, 12.1)  # -> DESCEND_HOLD
    sm.update(vs, small_tag, 13.2)  # -> DESCEND_OFFSET
    assert sm.state == LandingState.DESCEND_OFFSET

    # FSM returns use_offset=True — the node handles the actual transition
    state, info = sm.update(vs, small_tag, 14.0)
    assert state == LandingState.DESCEND_OFFSET
    assert info.get('use_offset') is True


def test_descend_offset_tag_lost():
    """DESCEND_OFFSET -> SEARCH when small tag lost for tag_lost_timeout."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    small_tag = MockTagStatus(detected=True, active_tag_id=1)
    sm.update(vs, small_tag, 10.0)
    sm.update(vs, small_tag, 12.1)  # -> DESCEND_HOLD
    sm.update(vs, small_tag, 13.2)  # -> DESCEND_OFFSET
    assert sm.state == LandingState.DESCEND_OFFSET

    no_tag = MockTagStatus(detected=False)
    sm.update(vs, no_tag, 14.0)  # Start lost timer
    sm.update(vs, no_tag, 16.0)  # 2s < 4s
    state, info = sm.update(vs, no_tag, 18.1)  # 4.1s > 4s
    assert state == LandingState.SEARCH
    assert info['action'] == 'tag_lost'


def test_descend_final_to_landed():
    """DESCEND_FINAL -> LANDED on landing detection."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    small_tag = MockTagStatus(detected=True, active_tag_id=1)
    sm.update(vs, small_tag, 10.0)
    sm.update(vs, small_tag, 12.1)  # -> DESCEND_HOLD
    sm.update(vs, small_tag, 13.2)  # -> DESCEND_OFFSET
    # Simulate offset achieved by setting state directly (as node would)
    sm.state = LandingState.DESCEND_FINAL

    landed = MockVehicleState(lat=LAT, lon=LON, alt_rel=0.05, armed=False, vz=0.0)
    state, info = sm.update(landed, small_tag, 20.0)
    assert state == LandingState.LANDED
    assert info['action'] == 'landed'


def test_search_pattern_finds_tag():
    """SMALL_TAG_SEARCH -> DESCEND_HOLD when small tag detected during search."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    # Trigger search pattern at floor altitude
    low_vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=1.5)
    large_tag = MockTagStatus(detected=True, active_tag_id=0)
    sm.update(low_vs, large_tag, 10.0)  # -> SMALL_TAG_SEARCH
    assert sm.state == LandingState.SMALL_TAG_SEARCH

    # Small tag appears during search
    small_tag = MockTagStatus(detected=True, active_tag_id=1)
    state, info = sm.update(low_vs, small_tag, 11.0)
    assert state == LandingState.DESCEND_HOLD
    assert info['action'] == 'small_tag_found'


def test_search_pattern_gives_up():
    """SMALL_TAG_SEARCH -> DESCEND_FINAL after full cycle with no detection."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    low_vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=1.5)
    large_tag = MockTagStatus(detected=True, active_tag_id=0)
    sm.update(low_vs, large_tag, 10.0)  # -> SMALL_TAG_SEARCH
    assert sm.state == LandingState.SMALL_TAG_SEARCH

    no_small = MockTagStatus(detected=True, active_tag_id=0)
    # Each direction: excursion (0.5/0.2 = 2.5s) + return (2.5s) = 5s
    # 4 directions = 20s total
    t = 10.0
    phase_duration = 0.5 / 0.2  # 2.5s

    for direction in range(4):
        # Excursion phase
        t += phase_duration + 0.01
        state, info = sm.update(low_vs, no_small, t)
        if state != LandingState.SMALL_TAG_SEARCH:
            break
        # Return phase
        t += phase_duration + 0.01
        state, info = sm.update(low_vs, no_small, t)
        if state != LandingState.SMALL_TAG_SEARCH:
            break

    assert state == LandingState.DESCEND_FINAL
    assert info['action'] == 'search_exhausted'


def test_global_timeout_from_descend_coarse():
    """DESCEND_COARSE -> ABORT_LAND on landing_timeout."""
    sm = LandingStateMachine(CONFIG)
    _to_descend_coarse(sm)

    vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=5.0)
    tag = MockTagStatus(detected=True, active_tag_id=0)
    state, info = sm.update(vs, tag, 62.0)  # start_time was ~1.0, so 62-1=61 > 60
    assert state == LandingState.ABORT_LAND
    assert info['action'] == 'timeout'


def test_global_timeout_from_descend_offset():
    """DESCEND_OFFSET -> ABORT_LAND on landing_timeout."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    small_tag = MockTagStatus(detected=True, active_tag_id=1)
    sm.update(vs, small_tag, 10.0)
    sm.update(vs, small_tag, 12.1)  # -> DESCEND_HOLD
    sm.update(vs, small_tag, 13.2)  # -> DESCEND_OFFSET
    assert sm.state == LandingState.DESCEND_OFFSET

    state, info = sm.update(vs, small_tag, 62.0)  # timeout
    assert state == LandingState.ABORT_LAND
    assert info['action'] == 'timeout'


# ---------------------------------------------------------------------------
# Velocity servo tag ID guard tests (ISS-012)
# ---------------------------------------------------------------------------

class MockTarget:
    def __init__(self, tag_id=0, position_x=0.1, position_y=0.1, position_valid=True):
        self.tag_id = tag_id
        self.position_x = position_x
        self.position_y = position_y
        self.position_valid = position_valid


def _apply_tag_guard(state, target, secondary_tag_id=1):
    """Replicates the tag ID guard logic from _velocity_servo."""
    expected_secondary = (state in (LandingState.DESCEND_HOLD,
                                    LandingState.DESCEND_OFFSET,
                                    LandingState.DESCEND_FINAL))
    if (expected_secondary and target is not None
            and target.tag_id != secondary_tag_id):
        target = None
    return target


def test_velocity_servo_rejects_wrong_tag_in_hold():
    """DESCEND_HOLD with target.tag_id=0 -> target treated as None (hold position)."""
    target = MockTarget(tag_id=0)
    result = _apply_tag_guard(LandingState.DESCEND_HOLD, target)
    assert result is None


def test_velocity_servo_accepts_correct_tag_in_hold():
    """DESCEND_HOLD with target.tag_id=1 -> normal PID output."""
    target = MockTarget(tag_id=1)
    result = _apply_tag_guard(LandingState.DESCEND_HOLD, target)
    assert result is target


# ---------------------------------------------------------------------------
# Preferred tag publish tests (ISS-012 circular dependency fix)
# ---------------------------------------------------------------------------

def _compute_preferred_tag(state, range_alt, slow_descent_altitude):
    """Replicates the preferred tag publish logic from _control_loop."""
    from dbvf_autonomy.precision_landing_node import compute_preferred_tag_id
    return compute_preferred_tag_id(state, range_alt, slow_descent_altitude)


def test_preferred_tag_secondary_during_late_descend_coarse():
    """DESCEND_COARSE with range_alt=1.8 (below 2.0m threshold) → preferred=1."""
    result = _compute_preferred_tag(LandingState.DESCEND_COARSE, 1.8, 2.0)
    assert result == 1


def test_preferred_tag_primary_during_early_descend_coarse():
    """DESCEND_COARSE with range_alt=3.0 (above 2.0m threshold) → preferred=0."""
    result = _compute_preferred_tag(LandingState.DESCEND_COARSE, 3.0, 2.0)
    assert result == 0


def test_preferred_tag_primary_when_range_alt_invalid():
    """DESCEND_COARSE with range_alt=-1.0 (invalid) → preferred=0."""
    result = _compute_preferred_tag(LandingState.DESCEND_COARSE, -1.0, 2.0)
    assert result == 0
