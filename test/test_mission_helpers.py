"""Tests for mission helper functions — pure math, no ROS2."""
import math

from dbvf_autonomy.mission_helpers import (
    ft_to_m,
    m_to_ft,
    haversine_distance_m,
    is_within_tolerance,
)


# ---------------------------------------------------------------------------
# Altitude conversion
# ---------------------------------------------------------------------------

def test_ft_to_m_35ft():
    assert abs(ft_to_m(35.0) - 10.668) < 0.001


def test_ft_to_m_zero():
    assert ft_to_m(0.0) == 0.0


def test_m_to_ft_10m():
    assert abs(m_to_ft(10.0) - 32.8084) < 0.01


def test_ft_m_roundtrip():
    """ft -> m -> ft should return the original value."""
    assert abs(m_to_ft(ft_to_m(100.0)) - 100.0) < 0.001


# ---------------------------------------------------------------------------
# Haversine distance
# ---------------------------------------------------------------------------

def test_haversine_same_point():
    d = haversine_distance_m(-35.363262, 149.165237, -35.363262, 149.165237)
    assert d < 0.01


def test_haversine_known_distance():
    """~111m per degree of latitude at any longitude."""
    d = haversine_distance_m(-35.0, 149.0, -35.001, 149.0)
    assert abs(d - 111.0) < 2.0  # Within 2m


def test_haversine_longitude_distance():
    """Longitude distance depends on latitude. At -35 deg, 1 deg lon ~ 91km."""
    d = haversine_distance_m(-35.0, 149.0, -35.0, 149.001)
    assert 80.0 < d < 100.0  # ~91m at lat -35


# ---------------------------------------------------------------------------
# Position tolerance
# ---------------------------------------------------------------------------

def test_within_tolerance_true():
    """Same point is within any positive tolerance."""
    assert is_within_tolerance(-35.363262, 149.165237,
                               -35.363262, 149.165237, 3.0)


def test_within_tolerance_false():
    """111m away is not within 3m tolerance."""
    assert not is_within_tolerance(-35.0, 149.0, -35.001, 149.0, 3.0)


def test_within_tolerance_boundary():
    """Point ~2m away is within 3m tolerance."""
    # ~2m north
    offset_lat = 2.0 / 111000.0
    assert is_within_tolerance(-35.0, 149.0,
                               -35.0 + offset_lat, 149.0, 3.0)
