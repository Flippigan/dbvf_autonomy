from dbvf_autonomy.mavlink_interface_node import (
    ARDUPILOT_MODE_MAP,
    build_landing_target_params,
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
