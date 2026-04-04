"""Tests for mission config validation — pure functions, no ROS2."""
from dbvf_autonomy.mission_helpers import validate_mission_config, DEFAULT_MISSION_CONFIG


def _make_config(**overrides):
    """Return a valid config dict with optional overrides."""
    cfg = dict(DEFAULT_MISSION_CONFIG)
    cfg.update(overrides)
    return cfg


def test_valid_config_passes():
    errors = validate_mission_config(_make_config())
    assert errors == []


def test_missing_required_gps_field():
    cfg = _make_config()
    del cfg['home_lat']
    errors = validate_mission_config(cfg)
    assert any('home_lat' in e for e in errors)


def test_invalid_drop_target():
    cfg = _make_config(drop_target='F3')
    errors = validate_mission_config(cfg)
    assert any('drop_target' in e for e in errors)


def test_negative_timeout():
    cfg = _make_config(mission_timeout_s=-1.0)
    errors = validate_mission_config(cfg)
    assert any('mission_timeout_s' in e for e in errors)


def test_zero_altitude_invalid():
    cfg = _make_config(transit_altitude_ft=0.0)
    errors = validate_mission_config(cfg)
    assert any('transit_altitude_ft' in e for e in errors)


def test_negative_payload_settle_time():
    cfg = _make_config(payload_settle_time_s=-1.0)
    errors = validate_mission_config(cfg)
    assert any('payload_settle_time_s' in e for e in errors)


def test_zero_payload_settle_time():
    cfg = _make_config(payload_settle_time_s=0.0)
    errors = validate_mission_config(cfg)
    assert any('payload_settle_time_s' in e for e in errors)


def test_valid_config_with_payload_params():
    cfg = _make_config(
        payload_servo_channel=0,
        payload_servo_pwm_hold=500,
        payload_servo_pwm_dispense=1000,
        payload_servo_pwm_drop=1500,
        payload_servo_pwm_pickup=2000,
        payload_servo_pwm_lock=2500,
        payload_settle_time_s=2.0,
    )
    errors = validate_mission_config(cfg)
    assert errors == []


def test_default_payload_servo_params_present():
    """DEFAULT_MISSION_CONFIG should include all payload_servo_* keys."""
    assert 'payload_servo_channel' in DEFAULT_MISSION_CONFIG
    assert 'payload_servo_pwm_hold' in DEFAULT_MISSION_CONFIG
    assert 'payload_servo_pwm_dispense' in DEFAULT_MISSION_CONFIG
    assert 'payload_servo_pwm_drop' in DEFAULT_MISSION_CONFIG
    assert 'payload_servo_pwm_pickup' in DEFAULT_MISSION_CONFIG
    assert 'payload_servo_pwm_lock' in DEFAULT_MISSION_CONFIG
    assert 'payload_settle_time_s' in DEFAULT_MISSION_CONFIG


def test_wa_offset_defaults_present():
    """DEFAULT_MISSION_CONFIG should include wa_offset_forward and wa_offset_right."""
    assert 'wa_offset_forward' in DEFAULT_MISSION_CONFIG
    assert 'wa_offset_right' in DEFAULT_MISSION_CONFIG
    assert DEFAULT_MISSION_CONFIG['wa_offset_forward'] == 0.0
    assert DEFAULT_MISSION_CONFIG['wa_offset_right'] == 0.0


def test_valid_config_with_wa_offsets():
    """Config with non-zero WA offsets should be valid."""
    cfg = _make_config(wa_offset_forward=0.12, wa_offset_right=-0.03)
    errors = validate_mission_config(cfg)
    assert errors == []
