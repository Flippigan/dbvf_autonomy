from dbvf_autonomy.mavlink_interface_node import (
    ARDUPILOT_MODE_MAP,
    RANGE_ALT_SENTINEL,
    VELOCITY_TYPE_MASK,
    build_landing_target_params,
    extract_rangefinder_distance,
)


def test_mode_mapping_common_modes():
    assert ARDUPILOT_MODE_MAP['GUIDED'] == 4
    assert ARDUPILOT_MODE_MAP['LAND'] == 9
    assert ARDUPILOT_MODE_MAP['RTL'] == 6
    assert ARDUPILOT_MODE_MAP['LOITER'] == 5
    assert ARDUPILOT_MODE_MAP['AUTO'] == 3
    assert ARDUPILOT_MODE_MAP['STABILIZE'] == 0


def test_landing_target_angles_only():
    params = build_landing_target_params(
        angle_x=0.1, angle_y=-0.05,
        position_x=0.0, position_y=0.0, position_z=0.0,
        position_valid=False,
        tag_size=0.6,
    )
    assert params['angle_x'] == 0.1
    assert params['angle_y'] == -0.05
    assert params['position_valid'] == 0
    assert params['size_x'] == 0.6
    assert params['size_y'] == 0.6
    assert params['distance'] == 0.0


def test_landing_target_with_position():
    params = build_landing_target_params(
        angle_x=0.1, angle_y=-0.05,
        position_x=1.0, position_y=0.5, position_z=3.0,
        position_valid=True,
        tag_size=0.15,
    )
    assert params['position_valid'] == 1
    assert params['x'] == 1.0
    assert params['y'] == 0.5
    assert params['z'] == 3.0
    assert params['size_x'] == 0.15


def test_range_alt_sentinel_before_data():
    """range_alt is -1.0 before first RANGEFINDER message received."""
    assert RANGE_ALT_SENTINEL == -1.0


def test_range_alt_populated_from_rangefinder():
    """range_alt populated correctly from RANGEFINDER MAVLink message."""
    class FakeRangefinderMsg:
        distance = 5.43

    assert extract_rangefinder_distance(FakeRangefinderMsg()) == 5.43


def test_velocity_type_mask():
    """Velocity-only type_mask has correct bits: pos ignored, vel used, accel ignored, yaw ignored."""
    # Bits 0-2: position (ignored = 1,1,1)
    assert VELOCITY_TYPE_MASK & 0b111 == 0b111
    # Bits 3-5: velocity (used = 0,0,0)
    assert (VELOCITY_TYPE_MASK >> 3) & 0b111 == 0b000
    # Bits 6-8: acceleration (ignored = 1,1,1)
    assert (VELOCITY_TYPE_MASK >> 6) & 0b111 == 0b111
    # Bit 10: yaw (ignored = 1)
    assert (VELOCITY_TYPE_MASK >> 10) & 0b1 == 0b1
    # Bit 11: yaw_rate (ignored = 1)
    assert (VELOCITY_TYPE_MASK >> 11) & 0b1 == 0b1


def test_velocity_frame():
    """MAV_FRAME_BODY_NED is frame 8 — body-relative NED."""
    from pymavlink import mavutil
    assert mavutil.mavlink.MAV_FRAME_BODY_NED == 8
