"""Arduino serial interface node — owns USB serial to Arduino servo controller."""
import serial

import rclpy
from rclpy.node import Node

from dbvf_msgs.srv import DoSetServo


def format_servo_command(servo_number, pwm):
    """Format a servo command string for the Arduino serial protocol.

    Protocol: S<servo_number>:<pwm>\n
    """
    return f'S{servo_number}:{pwm}\n'


def parse_servo_response(response):
    """Parse an Arduino serial response. Returns (success, message)."""
    response = response.strip()
    if response == 'OK':
        return True, 'OK'
    if response.startswith('ERR:'):
        return False, response[4:]
    return False, f'Unexpected response: {response}'


class ArduinoInterfaceNode(Node):
    def __init__(self):
        super().__init__('arduino_interface')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_timeout_s', 1.0)

        self._serial = None
        self._connect()

        self.create_service(
            DoSetServo, '/dbvf/arduino/set_servo', self._set_servo_cb)
        self.get_logger().info('Arduino interface node started')

    def _connect(self):
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('serial_timeout_s').value
        try:
            self._serial = serial.Serial(port, baud, timeout=timeout)
            self.get_logger().info(f'Serial connected: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial connection failed: {e}')
            self._serial = None

    def _set_servo_cb(self, request, response):
        if self._serial is None:
            self._connect()
        if self._serial is None:
            response.success = False
            response.message = 'Serial port unavailable'
            return response

        cmd = format_servo_command(request.servo_number, request.pwm)
        try:
            self._serial.write(cmd.encode('ascii'))
            raw = self._serial.readline().decode('ascii')
            if not raw:
                response.success = False
                response.message = 'Serial timeout'
                return response
            success, msg = parse_servo_response(raw)
            response.success = success
            response.message = msg
        except (serial.SerialException, UnicodeDecodeError) as e:
            self.get_logger().warn(f'Serial error: {e}')
            self._serial = None
            response.success = False
            response.message = f'Serial error: {e}'
        return response


def main():
    rclpy.init()
    node = ArduinoInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._serial:
            node._serial.close()
        node.destroy_node()
        rclpy.shutdown()
