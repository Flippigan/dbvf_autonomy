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
