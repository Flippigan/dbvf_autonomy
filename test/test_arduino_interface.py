"""Tests for Arduino interface — serial protocol pure functions."""
from dbvf_autonomy.arduino_interface_node import format_servo_command, parse_servo_response


def test_format_servo_command_basic():
    assert format_servo_command(1, 1100) == 'S1:1100\n'


def test_format_servo_command_different_values():
    assert format_servo_command(3, 1500) == 'S3:1500\n'


def test_parse_response_ok():
    success, msg = parse_servo_response('OK\n')
    assert success is True
    assert msg == 'OK'


def test_parse_response_ok_no_newline():
    success, msg = parse_servo_response('OK')
    assert success is True
    assert msg == 'OK'


def test_parse_response_error():
    success, msg = parse_servo_response('ERR:invalid servo\n')
    assert success is False
    assert msg == 'invalid servo'


def test_parse_response_unexpected():
    success, msg = parse_servo_response('WHAT\n')
    assert success is False
    assert 'Unexpected' in msg
