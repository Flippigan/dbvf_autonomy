"""Tests for global descent rate clamping near ground (ISS-011)."""
from dbvf_autonomy.precision_landing_node import clamp_descent_rate


# Default test parameters
SLOW_ALT = 2.0    # slow_descent_altitude
SLOW_RATE = 0.1   # slow_descent_rate (m/s downward)


def test_no_clamp_above_threshold():
    """Descent rate unchanged when rangefinder reads above threshold."""
    assert clamp_descent_rate(0.3, 5.0, SLOW_ALT, SLOW_RATE) == 0.3


def test_clamp_at_threshold():
    """Descent rate clamped when rangefinder reads exactly at threshold."""
    assert clamp_descent_rate(0.3, 2.0, SLOW_ALT, SLOW_RATE) == SLOW_RATE


def test_clamp_below_threshold():
    """Descent rate clamped when rangefinder reads below threshold."""
    assert clamp_descent_rate(0.3, 1.0, SLOW_ALT, SLOW_RATE) == SLOW_RATE


def test_no_clamp_already_slow():
    """Descent rate unchanged when already at or below slow rate."""
    assert clamp_descent_rate(0.05, 1.0, SLOW_ALT, SLOW_RATE) == 0.05


def test_no_clamp_exact_slow_rate():
    """Descent rate unchanged when exactly equal to slow rate."""
    assert clamp_descent_rate(0.1, 1.0, SLOW_ALT, SLOW_RATE) == 0.1


def test_no_clamp_zero_vz():
    """Zero vz (hold altitude) is not clamped."""
    assert clamp_descent_rate(0.0, 1.0, SLOW_ALT, SLOW_RATE) == 0.0


def test_no_clamp_ascending():
    """Negative vz (ascending) is not clamped."""
    assert clamp_descent_rate(-0.5, 1.0, SLOW_ALT, SLOW_RATE) == -0.5


def test_no_clamp_invalid_rangefinder():
    """No clamp when rangefinder has no valid reading (sentinel -1.0)."""
    assert clamp_descent_rate(0.3, -1.0, SLOW_ALT, SLOW_RATE) == 0.3
