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


# ---------------------------------------------------------------------------
# Mission config defaults and validation
# ---------------------------------------------------------------------------

_REQUIRED_GPS_KEYS = [
    'home_lat', 'home_lon',
    'landing_lat', 'landing_lon',
    'wa_lat', 'wa_lon',
    'f1_lat', 'f1_lon',
    'f2_lat', 'f2_lon',
]

_REQUIRED_POSITIVE = [
    'transit_altitude_ft',
    'position_tolerance_m',
    'takeoff_complete_alt_ft',
    'mission_timeout_s',
    'heartbeat_loss_timeout_s',
    'service_call_timeout_s',
    'guided_resend_interval_s',
    'drop_settle_time_s',
    'pickup_settle_time_s',
]

DEFAULT_MISSION_CONFIG = {
    # GPS coordinates (simulation defaults)
    'home_lat': -35.3632621,
    'home_lon': 149.1652374,
    'landing_lat': -35.3640000,
    'landing_lon': 149.1652374,
    'wa_lat': -35.3632531,
    'wa_lon': 149.1657896,
    'f1_lat': -35.3650000,
    'f1_lon': 149.1652374,
    'f2_lat': -35.3660000,
    'f2_lon': 149.1652374,

    # Flight parameters
    'transit_altitude_ft': 35.0,
    'position_tolerance_m': 3.0,
    'takeoff_complete_alt_ft': 33.0,

    # Payload servo
    'drop_servo_number': 9,
    'drop_servo_pwm_release': 1100,
    'drop_servo_pwm_hold': 1500,
    'drop_settle_time_s': 2.0,

    # WA reload servo (Arduino)
    'pickup_servo_number': 1,
    'pickup_servo_pwm_release': 1100,
    'pickup_servo_pwm_pickup': 1500,
    'pickup_settle_time_s': 2.0,

    # WA precision landing offset (camera-to-mechanism)
    'wa_offset_forward': 0.0,
    'wa_offset_right': 0.0,

    # Drop zone target
    'drop_target': 'F1',

    # Safety
    'mission_timeout_s': 540.0,
    'heartbeat_loss_timeout_s': 5.0,
    'service_call_timeout_s': 5.0,
    'guided_resend_interval_s': 1.0,

    # Altitude source
    'prefer_rangefinder': True,
    'rangefinder_max_m': 30.0,
}


def validate_mission_config(config):
    """Validate a mission config dict. Returns a list of error strings (empty = valid)."""
    errors = []

    for key in _REQUIRED_GPS_KEYS:
        if key not in config:
            errors.append(f"Missing required GPS field: {key}")

    for key in _REQUIRED_POSITIVE:
        if key in config and config[key] <= 0.0:
            errors.append(f"{key} must be positive, got {config[key]}")

    if config.get('drop_target') not in ('F1', 'F2'):
        errors.append(f"drop_target must be 'F1' or 'F2', got {config.get('drop_target')!r}")

    return errors
