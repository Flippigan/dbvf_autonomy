import math
from dbvf_autonomy.tag_detector_adapter_node import compute_angles


def test_center_pixel_gives_zero_angles():
    ax, ay = compute_angles(320.0, 240.0, cx=320.0, cy=240.0, fx=400.0, fy=400.0)
    assert abs(ax) < 1e-10
    assert abs(ay) < 1e-10


def test_right_offset_positive_angle_x():
    ax, ay = compute_angles(520.0, 240.0, cx=320.0, cy=240.0, fx=400.0, fy=400.0)
    assert ax > 0
    assert abs(ay) < 1e-10


def test_down_offset_positive_angle_y():
    ax, ay = compute_angles(320.0, 440.0, cx=320.0, cy=240.0, fx=400.0, fy=400.0)
    assert abs(ax) < 1e-10
    assert ay > 0


def test_angle_magnitude():
    ax, _ = compute_angles(520.0, 240.0, cx=320.0, cy=240.0, fx=400.0, fy=400.0)
    expected = math.atan2(200.0, 400.0)
    assert abs(ax - expected) < 1e-10


def test_left_offset_negative_angle_x():
    ax, _ = compute_angles(120.0, 240.0, cx=320.0, cy=240.0, fx=400.0, fy=400.0)
    assert ax < 0
