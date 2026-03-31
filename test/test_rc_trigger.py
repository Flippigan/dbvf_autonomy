from dbvf_autonomy.mavlink_interface_node import (
    detect_rc_rising_edge,
    get_rc_channel_pwm,
)


# -- Rising edge detection ---------------------------------------------------

def test_rising_edge_fires():
    """Trigger fires when PWM crosses from below to at/above threshold."""
    assert detect_rc_rising_edge(1800, 1000, 1700) is True


def test_sustained_high_does_not_refire():
    """No trigger when PWM stays above threshold."""
    assert detect_rc_rising_edge(1800, 1800, 1700) is False


def test_below_threshold_does_not_fire():
    """No trigger when PWM is below threshold."""
    assert detect_rc_rising_edge(1600, 1000, 1700) is False


def test_falling_edge_does_not_fire():
    """No trigger when PWM drops from above to below threshold."""
    assert detect_rc_rising_edge(1000, 1800, 1700) is False


def test_no_trigger_on_first_reading():
    """First reading (prev=None) records but does not trigger -- boot safety."""
    assert detect_rc_rising_edge(1800, None, 1700) is False


def test_retrigger_after_reset():
    """Trigger fires again after channel goes low then high."""
    # First: high (from low) -> fires
    assert detect_rc_rising_edge(1800, 1000, 1700) is True
    # Channel goes low -- no trigger
    assert detect_rc_rising_edge(1000, 1800, 1700) is False
    # Channel goes high again -- fires
    assert detect_rc_rising_edge(1800, 1000, 1700) is True


def test_exact_threshold_fires():
    """PWM exactly at threshold counts as high."""
    assert detect_rc_rising_edge(1700, 1000, 1700) is True


def test_one_below_threshold_does_not_fire():
    """PWM one below threshold does not fire."""
    assert detect_rc_rising_edge(1699, 1000, 1700) is False


def test_configurable_threshold():
    """Custom threshold values are respected."""
    # Threshold 1500: 1600 is above
    assert detect_rc_rising_edge(1600, 1000, 1500) is True
    # Threshold 1500: 1400 is below
    assert detect_rc_rising_edge(1400, 1000, 1500) is False


# -- Channel PWM extraction --------------------------------------------------

def test_get_channel_14():
    """Extract channel 14 PWM from RC_CHANNELS message."""
    class FakeMsg:
        chan14_raw = 1800
    assert get_rc_channel_pwm(FakeMsg(), 14) == 1800


def test_get_channel_15():
    """Extract channel 15 PWM from RC_CHANNELS message."""
    class FakeMsg:
        chan15_raw = 1200
    assert get_rc_channel_pwm(FakeMsg(), 15) == 1200


def test_get_channel_1():
    """Extract channel 1 PWM from RC_CHANNELS message."""
    class FakeMsg:
        chan1_raw = 1500
    assert get_rc_channel_pwm(FakeMsg(), 1) == 1500
