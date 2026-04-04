"""Tests for estimate_tag_position() — homography-based pose estimation."""
import math
from dbvf_autonomy.tag_detector_adapter_node import estimate_tag_position, estimate_tag_yaw, camera_to_body

# Camera intrinsics used across tests
FX = FY = 400.0
CX, CY = 320.0, 240.0

# Camera-to-body transforms for yaw tests
# Identity: body axes = camera axes
IDENTITY_TRANSFORM = [(0, 1.0), (1, 1.0), (2, 1.0)]
# Default drone config: body_x = -cam_y, body_y = cam_x, body_z = cam_z
DEFAULT_TRANSFORM = [(1, -1.0), (0, 1.0), (2, 1.0)]


def _build_homography(tx, ty, tz, fx, fy, cx, cy, tag_size):
    """Build the homography matrix for a tag at (tx, ty, tz) in camera frame.

    Assumes tag is parallel to image plane (no rotation).
    Returns 9-element row-major list matching AprilTagDetection.homography.
    """
    hs = tag_size / 2.0
    # H = K * [r1*hs, r2*hs, t]  with R=I, normalised so H[8]=1
    # = [[fx*hs, 0, fx*tx + cx*tz], [0, fy*hs, fy*ty + cy*tz], [0, 0, tz]]
    # Divide through by tz to normalise H[8] = 1
    return [
        fx * hs / tz, 0.0,          fx * tx / tz + cx,
        0.0,          fy * hs / tz, fy * ty / tz + cy,
        0.0,          0.0,          1.0,
    ]


def _build_rotated_homography(tx, ty, tz, theta, fx, fy, cx, cy, tag_size):
    """Build homography for a tag at (tx,ty,tz) rotated by theta around optical axis.

    theta is the in-plane rotation of the tag in the camera frame (radians).
    With theta=0, this is equivalent to _build_homography (tag parallel to image plane).
    """
    hs = tag_size / 2.0
    c, s = math.cos(theta), math.sin(theta)
    return [
        fx * c * hs / tz, -fx * s * hs / tz, fx * tx / tz + cx,
        fy * s * hs / tz,  fy * c * hs / tz, fy * ty / tz + cy,
        0.0,               0.0,               1.0,
    ]


def test_tag_centered_below():
    """Tag at (0, 0, 5) should return position (0, 0, 5)."""
    h = _build_homography(0.0, 0.0, 5.0, FX, FY, CX, CY, 0.15)
    x, y, z = estimate_tag_position(h, FX, FY, CX, CY, 0.15)
    assert abs(x) < 1e-6
    assert abs(y) < 1e-6
    assert abs(z - 5.0) < 1e-6


def test_tag_offset():
    """Tag at (1.0, 0.5, 5.0) should return that position."""
    h = _build_homography(1.0, 0.5, 5.0, FX, FY, CX, CY, 0.15)
    x, y, z = estimate_tag_position(h, FX, FY, CX, CY, 0.15)
    assert abs(x - 1.0) < 1e-6
    assert abs(y - 0.5) < 1e-6
    assert abs(z - 5.0) < 1e-6


def test_close_range():
    """Tag at (0, 0, 1.5) — close range typical of final descent."""
    h = _build_homography(0.0, 0.0, 1.5, FX, FY, CX, CY, 0.15)
    x, y, z = estimate_tag_position(h, FX, FY, CX, CY, 0.15)
    assert abs(x) < 1e-6
    assert abs(y) < 1e-6
    assert abs(z - 1.5) < 1e-6


def test_secondary_tag_size():
    """Works correctly with the smaller secondary tag (0.05m)."""
    h = _build_homography(0.2, -0.1, 2.0, FX, FY, CX, CY, 0.05)
    x, y, z = estimate_tag_position(h, FX, FY, CX, CY, 0.05)
    assert abs(x - 0.2) < 1e-6
    assert abs(y - (-0.1)) < 1e-6
    assert abs(z - 2.0) < 1e-6


def test_homography_scale_invariance():
    """Result should be the same regardless of homography scaling."""
    h_norm = _build_homography(0.5, 0.3, 4.0, FX, FY, CX, CY, 0.15)
    h_scaled = [v * 7.0 for v in h_norm]
    x1, y1, z1 = estimate_tag_position(h_norm, FX, FY, CX, CY, 0.15)
    x2, y2, z2 = estimate_tag_position(h_scaled, FX, FY, CX, CY, 0.15)
    assert abs(x1 - x2) < 1e-6
    assert abs(y1 - y2) < 1e-6
    assert abs(z1 - z2) < 1e-6


# --- estimate_tag_yaw tests ---

def test_yaw_zero_rotation_identity_transform():
    """Tag with no rotation, identity cam-to-body → yaw = 0."""
    h = _build_rotated_homography(0.0, 0.0, 5.0, 0.0, FX, FY, CX, CY, 0.15)
    yaw = estimate_tag_yaw(h, FX, FY, CX, CY, IDENTITY_TRANSFORM)
    assert abs(yaw - 0.0) < 1e-6


def test_yaw_45_degrees_identity_transform():
    """Tag rotated 45° in camera frame, identity transform → yaw = π/4."""
    h = _build_rotated_homography(0.0, 0.0, 5.0, math.pi / 4, FX, FY, CX, CY, 0.15)
    yaw = estimate_tag_yaw(h, FX, FY, CX, CY, IDENTITY_TRANSFORM)
    assert abs(yaw - math.pi / 4) < 1e-6


def test_yaw_negative_90_identity_transform():
    """Tag rotated -90° in camera frame, identity transform → yaw = -π/2."""
    h = _build_rotated_homography(0.0, 0.0, 5.0, -math.pi / 2, FX, FY, CX, CY, 0.15)
    yaw = estimate_tag_yaw(h, FX, FY, CX, CY, IDENTITY_TRANSFORM)
    assert abs(yaw - (-math.pi / 2)) < 1e-6


def test_yaw_zero_rotation_default_transform():
    """Tag with no rotation, default drone transform → yaw = π/2.

    Default: body_x = -cam_y, body_y = cam_x. The 90° axis remap adds π/2
    to the camera-frame angle.
    """
    h = _build_rotated_homography(0.0, 0.0, 5.0, 0.0, FX, FY, CX, CY, 0.15)
    yaw = estimate_tag_yaw(h, FX, FY, CX, CY, DEFAULT_TRANSFORM)
    assert abs(yaw - math.pi / 2) < 1e-6


def test_yaw_45_degrees_default_transform():
    """Tag rotated 45° in camera frame, default transform → yaw = 3π/4."""
    h = _build_rotated_homography(0.0, 0.0, 5.0, math.pi / 4, FX, FY, CX, CY, 0.15)
    yaw = estimate_tag_yaw(h, FX, FY, CX, CY, DEFAULT_TRANSFORM)
    assert abs(yaw - 3 * math.pi / 4) < 1e-6


def test_yaw_scale_invariance():
    """Yaw should be identical regardless of homography scaling."""
    h_norm = _build_rotated_homography(0.5, 0.3, 4.0, math.pi / 6, FX, FY, CX, CY, 0.15)
    h_scaled = [v * 7.0 for v in h_norm]
    yaw1 = estimate_tag_yaw(h_norm, FX, FY, CX, CY, IDENTITY_TRANSFORM)
    yaw2 = estimate_tag_yaw(h_scaled, FX, FY, CX, CY, IDENTITY_TRANSFORM)
    assert abs(yaw1 - yaw2) < 1e-6


def test_yaw_independent_of_translation():
    """Yaw depends only on rotation, not on tag position."""
    yaw_a = estimate_tag_yaw(
        _build_rotated_homography(0.0, 0.0, 5.0, math.pi / 3, FX, FY, CX, CY, 0.15),
        FX, FY, CX, CY, IDENTITY_TRANSFORM)
    yaw_b = estimate_tag_yaw(
        _build_rotated_homography(2.0, -1.0, 3.0, math.pi / 3, FX, FY, CX, CY, 0.15),
        FX, FY, CX, CY, IDENTITY_TRANSFORM)
    assert abs(yaw_a - yaw_b) < 1e-6
