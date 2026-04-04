import pytest

from dbvf_autonomy.mavlink_interface_node import build_connection_string


def test_serial_connection():
    conn_str, kwargs = build_connection_string(
        'serial', '/dev/ttyTHS1', 921600,
        '127.0.0.1', 5762, '127.0.0.1', 14550,
    )
    assert conn_str == '/dev/ttyTHS1'
    assert kwargs == {'baud': 921600}


def test_tcp_connection():
    conn_str, kwargs = build_connection_string(
        'tcp', '/dev/ttyTHS1', 921600,
        '127.0.0.1', 5762, '127.0.0.1', 14550,
    )
    assert conn_str == 'tcp:127.0.0.1:5762'
    assert kwargs == {}


def test_udp_connection():
    conn_str, kwargs = build_connection_string(
        'udp', '/dev/ttyTHS1', 921600,
        '127.0.0.1', 5762, '0.0.0.0', 14550,
    )
    assert conn_str == 'udpin:0.0.0.0:14550'
    assert kwargs == {}


def test_unknown_connection_type_raises():
    with pytest.raises(ValueError, match='Unknown connection_type'):
        build_connection_string(
            'bluetooth', '/dev/ttyTHS1', 921600,
            '127.0.0.1', 5762, '127.0.0.1', 14550,
        )


def test_serial_custom_device_and_baud():
    conn_str, kwargs = build_connection_string(
        'serial', '/dev/ttyUSB0', 57600,
        '127.0.0.1', 5762, '127.0.0.1', 14550,
    )
    assert conn_str == '/dev/ttyUSB0'
    assert kwargs == {'baud': 57600}


def test_tcp_custom_host_and_port():
    conn_str, kwargs = build_connection_string(
        'tcp', '/dev/ttyTHS1', 921600,
        '192.168.1.10', 5763, '127.0.0.1', 14550,
    )
    assert conn_str == 'tcp:192.168.1.10:5763'
    assert kwargs == {}


def test_udp_custom_host_and_port():
    conn_str, kwargs = build_connection_string(
        'udp', '/dev/ttyTHS1', 921600,
        '127.0.0.1', 5762, '192.168.1.10', 14551,
    )
    assert conn_str == 'udpin:192.168.1.10:14551'
    assert kwargs == {}
