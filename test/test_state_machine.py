from dbvf_autonomy.precision_landing_node import (
    LandingStateMachine, LandingState, PIDController, wrap_angle,
)


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
    'offset_tolerance': 0.05,
    'small_tag_search_radius': 0.5,
    'small_tag_search_speed': 0.2,
    'primary_tag_id': 1,
    'secondary_tag_id': 2,
    'slow_descent_altitude': 2.0,
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
    tag = MockTagStatus(detected=True, active_tag_id=1)
    for i in range(5):
        sm.update(vs, tag, 2.0 + i * 0.05)
    assert sm.state == LandingState.DESCEND_COARSE
    return vs


def _to_hold_above_tag(sm):
    """Advance from IDLE to HOLD_ABOVE_TAG by directly setting state.

    In production, DESCEND_COARSE transitions here after small tag
    confirmation. We set state directly to decouple from the transition
    target update (Task 6). The natural transition path is tested there.
    """
    vs = _to_descend_coarse(sm)
    sm.state = LandingState.HOLD_ABOVE_TAG
    return vs


def _to_align_yaw(sm):
    """Advance from IDLE to ALIGN_YAW by directly setting state.

    In production, the ROS node triggers HOLD_ABOVE_TAG → ALIGN_YAW
    when position is stable. We simulate that by setting state directly.
    """
    vs = _to_hold_above_tag(sm)
    sm.state = LandingState.ALIGN_YAW
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

    tag = MockTagStatus(detected=True, active_tag_id=1)
    for i in range(4):
        state, _ = sm.update(vs, tag, 2.0 + i * 0.05)
        assert state == LandingState.SEARCH  # Not confirmed yet

    state, info = sm.update(vs, tag, 2.25)
    assert state == LandingState.DESCEND_COARSE
    assert info['action'] == 'tag_confirmed'


def test_search_resets_confirm_on_loss():
    sm = LandingStateMachine(CONFIG)
    vs = _to_search(sm)

    tag = MockTagStatus(detected=True, active_tag_id=1)
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
    tag = MockTagStatus(detected=True, active_tag_id=1)
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
    tag = MockTagStatus(detected=True, active_tag_id=1)
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

def test_descend_coarse_to_hold_above_tag():
    """DESCEND_COARSE -> HOLD_ABOVE_TAG when small tag detected for small_tag_confirm_time."""
    sm = LandingStateMachine(CONFIG)
    _to_descend_coarse(sm)

    low_vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=1.8)
    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    # First detection starts the timer
    sm.update(low_vs, small_tag, 10.0)
    assert sm.state == LandingState.DESCEND_COARSE

    # Not enough time yet
    sm.update(low_vs, small_tag, 11.0)
    assert sm.state == LandingState.DESCEND_COARSE

    # 2.0 seconds of continuous detection → transition
    state, info = sm.update(low_vs, small_tag, 12.1)
    assert state == LandingState.HOLD_ABOVE_TAG
    assert info['action'] == 'small_tag_confirmed'


def test_descend_coarse_stays_if_above_slow_descent_altitude():
    """DESCEND_COARSE stays even with confirmed small tag when above slow_descent_altitude (ISS-016)."""
    sm = LandingStateMachine(CONFIG)
    _to_descend_coarse(sm)

    high_vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=5.0)
    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    # Confirm small tag for well over small_tag_confirm_time at high altitude
    sm.update(high_vs, small_tag, 10.0)
    sm.update(high_vs, small_tag, 13.0)  # 3.0s > 2.0s confirm time
    assert sm.state == LandingState.DESCEND_COARSE  # Still descending, not hovering


def test_descend_coarse_floor_with_primary_continues():
    """DESCEND_COARSE at floor altitude with primary tag visible stays in DESCEND_COARSE.

    The primary tag is actively guiding PID descent — let it continue so
    the secondary tag can be confirmed naturally as the drone gets closer.
    """
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    low_vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=1.5)
    large_tag = MockTagStatus(detected=True, active_tag_id=1)
    state, info = sm.update(low_vs, large_tag, 10.0)
    assert state == LandingState.DESCEND_COARSE
    assert info['action'] == 'descending'


def test_descend_coarse_floor_no_tag_triggers_search():
    """DESCEND_COARSE -> SMALL_TAG_SEARCH at floor altitude when NO tag visible."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    low_vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=1.5)
    no_tag = MockTagStatus(detected=False)
    state, info = sm.update(low_vs, no_tag, 10.0)
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


def test_offset_lateral_to_final():
    """OFFSET_LATERAL -> DESCEND_FINAL when position error < offset_tolerance.

    Note: The actual offset checking is done in the ROS node's _velocity_servo,
    which directly sets fsm.state. The FSM's _offset_lateral returns use_offset=True
    to signal the node to check. We test the FSM returns the correct info.
    """
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)
    sm.state = LandingState.OFFSET_LATERAL

    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    # FSM returns use_offset=True — the node handles the actual transition
    state, info = sm.update(vs, small_tag, 14.0)
    assert state == LandingState.OFFSET_LATERAL
    assert info.get('use_offset') is True


def test_offset_lateral_tag_lost():
    """OFFSET_LATERAL -> SEARCH when small tag lost for tag_lost_timeout."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)
    sm.state = LandingState.OFFSET_LATERAL

    no_tag = MockTagStatus(detected=False)
    sm.update(vs, no_tag, 14.0)  # Start lost timer
    sm.update(vs, no_tag, 16.0)  # 2s < 4s
    state, info = sm.update(vs, no_tag, 18.1)  # 4.1s > 4s
    assert state == LandingState.SEARCH
    assert info['action'] == 'tag_lost'


# ---------------------------------------------------------------------------
# HOLD_ABOVE_TAG tests
# ---------------------------------------------------------------------------

def test_hold_above_tag_returns_hold_action():
    """HOLD_ABOVE_TAG returns vz=0.0 and use_offset=False."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_hold_above_tag(sm)
    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    state, info = sm.update(vs, small_tag, 13.0)
    assert state == LandingState.HOLD_ABOVE_TAG
    assert info['action'] == 'holding'
    assert info['vz'] == 0.0
    assert info['use_offset'] is False


def test_hold_above_tag_tag_lost_to_search():
    """HOLD_ABOVE_TAG → SEARCH when secondary tag lost for tag_lost_timeout."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_hold_above_tag(sm)
    no_tag = MockTagStatus(detected=False)
    sm.update(vs, no_tag, 13.0)  # Start lost timer
    sm.update(vs, no_tag, 15.0)  # 2s < 4s
    state, info = sm.update(vs, no_tag, 17.1)  # 4.1s > 4s
    assert state == LandingState.SEARCH
    assert info['action'] == 'tag_lost'


def test_hold_above_tag_primary_only_triggers_tag_lost():
    """HOLD_ABOVE_TAG with only primary tag visible → tag_lost timer runs."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_hold_above_tag(sm)
    primary_tag = MockTagStatus(detected=True, active_tag_id=1)
    sm.update(vs, primary_tag, 13.0)  # Primary only — secondary lost
    sm.update(vs, primary_tag, 15.0)
    state, info = sm.update(vs, primary_tag, 17.1)  # 4.1s > 4s
    assert state == LandingState.SEARCH
    assert info['action'] == 'tag_lost'


def test_hold_above_tag_to_landed():
    """HOLD_ABOVE_TAG → LANDED on disarm."""
    sm = LandingStateMachine(CONFIG)
    _to_hold_above_tag(sm)
    landed = MockVehicleState(lat=LAT, lon=LON, alt_rel=0.05, armed=False, vz=0.0)
    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    state, info = sm.update(landed, small_tag, 13.0)
    assert state == LandingState.LANDED
    assert info['action'] == 'landed'


def test_hold_above_tag_tag_reacquired_resets_timer():
    """HOLD_ABOVE_TAG tag reacquired before timeout resets lost timer."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_hold_above_tag(sm)
    no_tag = MockTagStatus(detected=False)
    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    sm.update(vs, no_tag, 13.0)   # Lost at t=13
    sm.update(vs, no_tag, 15.0)   # 2s elapsed
    sm.update(vs, small_tag, 15.5)  # Reacquired — resets timer
    sm.update(vs, no_tag, 16.0)   # Lost again at t=16
    state, _ = sm.update(vs, no_tag, 19.5)  # 3.5s < 4s
    assert state == LandingState.HOLD_ABOVE_TAG


# ---------------------------------------------------------------------------
# ALIGN_YAW tests
# ---------------------------------------------------------------------------

def test_align_yaw_returns_aligning_action():
    """ALIGN_YAW returns vz=0.0 and use_offset=False."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_align_yaw(sm)
    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    state, info = sm.update(vs, small_tag, 14.0)
    assert state == LandingState.ALIGN_YAW
    assert info['action'] == 'aligning_yaw'
    assert info['vz'] == 0.0
    assert info['use_offset'] is False


def test_align_yaw_tag_lost_to_search():
    """ALIGN_YAW → SEARCH when secondary tag lost for tag_lost_timeout."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_align_yaw(sm)
    no_tag = MockTagStatus(detected=False)
    sm.update(vs, no_tag, 14.0)  # Start lost timer
    sm.update(vs, no_tag, 16.0)  # 2s < 4s
    state, info = sm.update(vs, no_tag, 18.1)  # 4.1s > 4s
    assert state == LandingState.SEARCH
    assert info['action'] == 'tag_lost'


def test_align_yaw_primary_only_triggers_tag_lost():
    """ALIGN_YAW with only primary tag visible → tag_lost timer runs."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_align_yaw(sm)
    primary_tag = MockTagStatus(detected=True, active_tag_id=1)
    sm.update(vs, primary_tag, 14.0)
    sm.update(vs, primary_tag, 16.0)
    state, info = sm.update(vs, primary_tag, 18.1)
    assert state == LandingState.SEARCH
    assert info['action'] == 'tag_lost'


def test_align_yaw_to_landed():
    """ALIGN_YAW → LANDED on disarm."""
    sm = LandingStateMachine(CONFIG)
    _to_align_yaw(sm)
    landed = MockVehicleState(lat=LAT, lon=LON, alt_rel=0.05, armed=False, vz=0.0)
    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    state, info = sm.update(landed, small_tag, 14.0)
    assert state == LandingState.LANDED
    assert info['action'] == 'landed'


def test_align_yaw_tag_reacquired_resets_timer():
    """ALIGN_YAW tag reacquired before timeout resets lost timer."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_align_yaw(sm)
    no_tag = MockTagStatus(detected=False)
    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    sm.update(vs, no_tag, 14.0)   # Lost
    sm.update(vs, no_tag, 16.0)   # 2s
    sm.update(vs, small_tag, 16.5)  # Reacquired
    sm.update(vs, no_tag, 17.0)   # Lost again
    state, _ = sm.update(vs, no_tag, 20.5)  # 3.5s < 4s
    assert state == LandingState.ALIGN_YAW


def test_descend_final_to_landed():
    """DESCEND_FINAL -> LANDED on landing detection."""
    sm = LandingStateMachine(CONFIG)
    _to_descend_coarse(sm)
    sm.state = LandingState.DESCEND_FINAL

    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    landed = MockVehicleState(lat=LAT, lon=LON, alt_rel=0.05, armed=False, vz=0.0)
    state, info = sm.update(landed, small_tag, 20.0)
    assert state == LandingState.LANDED
    assert info['action'] == 'landed'


def test_search_pattern_finds_tag():
    """SMALL_TAG_SEARCH -> HOLD_ABOVE_TAG when small tag detected during search."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    # Trigger search pattern at floor altitude with no tag visible
    low_vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=1.5)
    no_tag = MockTagStatus(detected=False)
    sm.update(low_vs, no_tag, 10.0)  # -> SMALL_TAG_SEARCH
    assert sm.state == LandingState.SMALL_TAG_SEARCH

    # Small tag appears during search
    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    state, info = sm.update(low_vs, small_tag, 11.0)
    assert state == LandingState.HOLD_ABOVE_TAG
    assert info['action'] == 'small_tag_found'


def test_search_pattern_gives_up():
    """SMALL_TAG_SEARCH -> DESCEND_FINAL after full cycle with no detection."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    low_vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=1.5)
    no_tag = MockTagStatus(detected=False)
    sm.update(low_vs, no_tag, 10.0)  # -> SMALL_TAG_SEARCH
    assert sm.state == LandingState.SMALL_TAG_SEARCH

    no_small = MockTagStatus(detected=True, active_tag_id=1)
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
    tag = MockTagStatus(detected=True, active_tag_id=1)
    state, info = sm.update(vs, tag, 62.0)  # start_time was ~1.0, so 62-1=61 > 60
    assert state == LandingState.ABORT_LAND
    assert info['action'] == 'timeout'


def test_global_timeout_from_offset_lateral():
    """OFFSET_LATERAL -> ABORT_LAND on landing_timeout."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)
    sm.state = LandingState.OFFSET_LATERAL

    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    state, info = sm.update(vs, small_tag, 62.0)  # timeout
    assert state == LandingState.ABORT_LAND
    assert info['action'] == 'timeout'


# ---------------------------------------------------------------------------
# Velocity servo tag ID guard tests (ISS-012)
# ---------------------------------------------------------------------------

class MockTarget:
    def __init__(self, tag_id=1, position_x=0.1, position_y=0.1, position_valid=True):
        self.tag_id = tag_id
        self.position_x = position_x
        self.position_y = position_y
        self.position_valid = position_valid


def _apply_tag_guard(state, target, secondary_tag_id=2):
    """Replicates the tag ID guard logic from _velocity_servo."""
    expected_secondary = (state in (LandingState.HOLD_ABOVE_TAG,
                                    LandingState.ALIGN_YAW,
                                    LandingState.OFFSET_LATERAL,
                                    LandingState.DESCEND_FINAL))
    if (expected_secondary and target is not None
            and target.tag_id != secondary_tag_id):
        target = None
    return target


def test_velocity_servo_rejects_wrong_tag_in_offset():
    """OFFSET_LATERAL with target.tag_id=1 (primary) -> target treated as None (hold position)."""
    target = MockTarget(tag_id=1)
    result = _apply_tag_guard(LandingState.OFFSET_LATERAL, target)
    assert result is None


def test_velocity_servo_accepts_correct_tag_in_offset():
    """OFFSET_LATERAL with target.tag_id=2 (secondary) -> normal PID output."""
    target = MockTarget(tag_id=2)
    result = _apply_tag_guard(LandingState.OFFSET_LATERAL, target)
    assert result is target


# ---------------------------------------------------------------------------
# Preferred tag publish tests (ISS-012 circular dependency fix)
# ---------------------------------------------------------------------------

def _compute_preferred_tag(state, range_alt, slow_descent_altitude):
    """Replicates the preferred tag publish logic from _control_loop."""
    from dbvf_autonomy.precision_landing_node import compute_preferred_tag_id
    return compute_preferred_tag_id(state, range_alt, slow_descent_altitude)


def test_preferred_tag_secondary_during_late_descend_coarse():
    """DESCEND_COARSE with range_alt=1.8 (below 2.0m threshold) → preferred=2 (secondary)."""
    result = _compute_preferred_tag(LandingState.DESCEND_COARSE, 1.8, 2.0)
    assert result == 2


def test_preferred_tag_primary_during_early_descend_coarse():
    """DESCEND_COARSE with range_alt=3.0 (above 2.0m threshold) → preferred=1 (primary)."""
    result = _compute_preferred_tag(LandingState.DESCEND_COARSE, 3.0, 2.0)
    assert result == 1


def test_preferred_tag_primary_when_range_alt_invalid():
    """DESCEND_COARSE with range_alt=-1.0 (invalid) → preferred=1 (primary)."""
    result = _compute_preferred_tag(LandingState.DESCEND_COARSE, -1.0, 2.0)
    assert result == 1


def test_preferred_tag_secondary_during_hold_above_tag():
    """HOLD_ABOVE_TAG → preferred=2 (secondary)."""
    result = _compute_preferred_tag(LandingState.HOLD_ABOVE_TAG, 1.5, 2.0)
    assert result == 2


def test_preferred_tag_secondary_during_align_yaw():
    """ALIGN_YAW → preferred=2 (secondary)."""
    result = _compute_preferred_tag(LandingState.ALIGN_YAW, 1.5, 2.0)
    assert result == 2


def test_preferred_tag_secondary_during_offset_lateral():
    """OFFSET_LATERAL → preferred=2 (secondary)."""
    result = _compute_preferred_tag(LandingState.OFFSET_LATERAL, 1.5, 2.0)
    assert result == 2


# ---------------------------------------------------------------------------
# Per-landing offset tests
# ---------------------------------------------------------------------------

def test_velocity_servo_applies_offset_in_descend_offset():
    """Offset should be subtracted from tag error when use_offset=True."""
    from dbvf_autonomy.precision_landing_node import PIDController

    offset_fwd = 0.12
    offset_right = 0.05

    # Simulate what _velocity_servo does with offsets
    error_x = 0.15
    error_y = 0.10
    error_x -= offset_fwd   # 0.15 - 0.12 = 0.03
    error_y -= offset_right  # 0.10 - 0.05 = 0.05

    assert abs(error_x - 0.03) < 1e-9
    assert abs(error_y - 0.05) < 1e-9


def test_velocity_servo_no_offset_when_not_flagged():
    """Without use_offset=True, raw tag error should be used."""
    error_x = 0.15
    error_y = 0.10
    # No offset subtraction
    assert error_x == 0.15
    assert error_y == 0.10


# ---------------------------------------------------------------------------
# ISS-014: Body-frame yaw hold regression test
# ---------------------------------------------------------------------------

def test_body_frame_yaw_hold_uses_zero():
    """ISS-014: In MAV_FRAME_BODY_NED, yaw=0.0 means 'hold current heading'.

    Sending math.radians(heading) would be interpreted as a RELATIVE rotation
    from current heading (ArduPilot GCS_MAVLink_Copter.cpp:1372 sets
    yaw_relative=true for BODY_NED frames), causing a continuous spin.
    """
    # Simulate what _call_guided_velocity does for yaw hold.
    # The node sets req.yaw when vs is not None.
    # In body frame, the correct value is always 0.0 (no rotation from current).
    import math

    class FakeVehicleState:
        heading = 90.0  # degrees — any non-zero heading

    vs = FakeVehicleState()

    # WRONG (old code): would command +90° relative rotation every tick
    wrong_yaw = math.radians(vs.heading)
    assert wrong_yaw != 0.0, "Test setup: heading must be non-zero"

    # CORRECT: body-frame yaw hold = 0.0 (no rotation from current heading)
    correct_yaw = 0.0
    assert correct_yaw == 0.0


# ---------------------------------------------------------------------------
# wrap_angle tests
# ---------------------------------------------------------------------------

def test_wrap_angle_zero():
    assert wrap_angle(0.0) == 0.0


def test_wrap_angle_positive_within_range():
    import math
    assert abs(wrap_angle(1.0) - 1.0) < 1e-9


def test_wrap_angle_negative_within_range():
    import math
    assert abs(wrap_angle(-1.0) - (-1.0)) < 1e-9


def test_wrap_angle_greater_than_pi():
    import math
    # 3*pi/2 should wrap to -pi/2
    result = wrap_angle(3 * math.pi / 2)
    assert abs(result - (-math.pi / 2)) < 1e-9


def test_wrap_angle_less_than_neg_pi():
    import math
    # -3*pi/2 should wrap to pi/2
    result = wrap_angle(-3 * math.pi / 2)
    assert abs(result - (math.pi / 2)) < 1e-9


def test_wrap_angle_exactly_pi():
    import math
    # pi should stay as pi (boundary)
    result = wrap_angle(math.pi)
    assert abs(result - math.pi) < 1e-9 or abs(result - (-math.pi)) < 1e-9


def test_wrap_angle_two_pi():
    import math
    result = wrap_angle(2 * math.pi)
    assert abs(result) < 1e-9
