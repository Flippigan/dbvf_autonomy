from dbvf_autonomy.tag_detector_adapter_node import camera_to_body, _parse_axis


def _default_transform():
    """Default camera-to-body: body_x=-cam_y, body_y=cam_x, body_z=cam_z."""
    return [
        _parse_axis('-y'),
        _parse_axis('x'),
        _parse_axis('z'),
    ]


def test_tag_directly_below():
    """Camera (0, 0, z) should map to body (0, 0, z)."""
    bx, by, bz = camera_to_body(0.0, 0.0, 3.0, _default_transform())
    assert abs(bx) < 1e-9
    assert abs(by) < 1e-9
    assert abs(bz - 3.0) < 1e-9


def test_tag_offset_camera_x():
    """Camera (+x, 0, z) → body (0, +y, z). Camera X maps to body Y."""
    bx, by, bz = camera_to_body(0.5, 0.0, 3.0, _default_transform())
    assert abs(bx) < 1e-9        # body_x = -cam_y = 0
    assert abs(by - 0.5) < 1e-9  # body_y = cam_x = 0.5
    assert abs(bz - 3.0) < 1e-9


def test_tag_offset_camera_y():
    """Camera (0, +y, z) → body (-y, 0, z). Camera Y maps to body -X."""
    bx, by, bz = camera_to_body(0.0, 0.5, 3.0, _default_transform())
    assert abs(bx - (-0.5)) < 1e-9  # body_x = -cam_y = -0.5
    assert abs(by) < 1e-9           # body_y = cam_x = 0
    assert abs(bz - 3.0) < 1e-9


def test_transform_roundtrip():
    """Known offsets produce expected body-frame values."""
    bx, by, bz = camera_to_body(1.0, -2.0, 5.0, _default_transform())
    # body_x = -cam_y = -(-2.0) = 2.0
    # body_y = cam_x = 1.0
    # body_z = cam_z = 5.0
    assert abs(bx - 2.0) < 1e-9
    assert abs(by - 1.0) < 1e-9
    assert abs(bz - 5.0) < 1e-9
