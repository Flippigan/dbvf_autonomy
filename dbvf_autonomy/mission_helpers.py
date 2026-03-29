"""Pure helper functions for the mission sequencer — no ROS2 dependencies."""
import math

# Conversion factor
_FT_PER_METER = 3.28084


def ft_to_m(feet):
    """Convert feet to meters."""
    return feet / _FT_PER_METER


def m_to_ft(meters):
    """Convert meters to feet."""
    return meters * _FT_PER_METER


def haversine_distance_m(lat1, lon1, lat2, lon2):
    """Haversine distance between two GPS points in meters."""
    R = 6371000.0  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2.0) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2.0) ** 2)
    return R * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def is_within_tolerance(lat1, lon1, lat2, lon2, tolerance_m):
    """Return True if the two GPS points are within tolerance_m meters."""
    return haversine_distance_m(lat1, lon1, lat2, lon2) < tolerance_m
