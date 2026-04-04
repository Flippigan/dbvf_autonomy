"""Interactive servo test node — exercises payload servos through mission phases.

Uses the exact same /dbvf/arduino/set_servo service path as the competition
mission sequencer. Loads PWM values from mission_params.yaml so the test
matches what will fly.

Usage (standalone):
    ros2 launch dbvf_autonomy servo_test.launch.py

Then follow the on-screen menu to step through each servo phase.
"""
import sys
import threading

import rclpy
from rclpy.node import Node

from dbvf_msgs.srv import DoSetServo


# Mission servo phases in competition order.
# Each tuple: (key, label, config_channel_key, config_pwm_key, description)
SERVO_PHASES = [
    ('dispense', '1) DROP_PAYLOAD — Dispense red payload',
     'payload_servo_channel', 'payload_servo_pwm_dispense',
     'Opens release mechanism to drop payload at F1/F2'),
    ('drop', '2) WA_DROP_OLD — Drop old payload at WA',
     'payload_servo_channel', 'payload_servo_pwm_drop',
     'Drops old red payload cradle before picking up new one'),
    ('pickup', '3) WA_PICKUP_READY — Open for crew loading',
     'payload_servo_channel', 'payload_servo_pwm_pickup',
     'Resets servo to magnetic pickup orientation'),
    ('lock', '4) WA_LOCK_PAYLOAD — Lock new yellow payload',
     'payload_servo_channel', 'payload_servo_pwm_lock',
     'Secures new payload via lock mechanism after landing'),
    ('dispense2', '5) DROP_PAYLOAD_2 — Dispense yellow payload',
     'payload_servo_channel', 'payload_servo_pwm_dispense',
     'Same as phase 1 — releases yellow payload at F1/F2'),
]

# Default PWM config matching mission_params.yaml keys
DEFAULT_CONFIG = {
    'payload_servo_channel': 0,
    'payload_servo_pwm_hold': 0,
    'payload_servo_pwm_dispense': 0,
    'payload_servo_pwm_drop': 0,
    'payload_servo_pwm_pickup': 0,
    'payload_servo_pwm_lock': 0,
}


class ServoTestNode(Node):
    def __init__(self):
        super().__init__('servo_test')

        # Declare parameters matching mission_params.yaml keys
        for key, default in DEFAULT_CONFIG.items():
            self.declare_parameter(key, default)

        # Load config from parameters (populated by launch file from YAML)
        self.cfg = {}
        for key in DEFAULT_CONFIG:
            self.cfg[key] = self.get_parameter(key).value

        # Service client — same path as mission sequencer
        self.cli = self.create_client(DoSetServo, '/dbvf/arduino/set_servo')

        self.get_logger().info('Servo test node started')
        self.get_logger().info(f'Config: channel={self.cfg["payload_servo_channel"]}, '
                               f'dispense={self.cfg["payload_servo_pwm_dispense"]}, '
                               f'drop={self.cfg["payload_servo_pwm_drop"]}, '
                               f'pickup={self.cfg["payload_servo_pwm_pickup"]}, '
                               f'lock={self.cfg["payload_servo_pwm_lock"]}')

        # Run interactive menu on a separate thread so rclpy.spin() keeps running
        self._menu_thread = threading.Thread(target=self._run_menu, daemon=True)
        self._menu_thread.start()

    def _call_servo(self, servo_number, pwm):
        """Call /dbvf/arduino/set_servo — identical to mission_sequencer_node."""
        if not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                'arduino/set_servo service unavailable — is arduino_interface_node running?')
            return False, 'Service unavailable'

        req = DoSetServo.Request()
        req.servo_number = servo_number
        req.pwm = pwm

        self.get_logger().info(f'Sending: servo={servo_number}, pwm={pwm}')
        future = self.cli.call_async(req)

        # Block until response (with timeout)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error('Service call timed out')
            return False, 'Timeout'

        result = future.result()
        if result.success:
            self.get_logger().info(f'OK: {result.message}')
        else:
            self.get_logger().warn(f'FAILED: {result.message}')
        return result.success, result.message

    def _run_menu(self):
        """Interactive terminal menu — runs on background thread."""
        # Wait briefly for node to fully initialize
        import time
        time.sleep(1.0)

        while rclpy.ok():
            self._print_menu()
            try:
                choice = input('\n> ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                print('\nExiting servo test.')
                rclpy.shutdown()
                return

            if choice in ('q', 'quit', 'exit'):
                print('Exiting servo test.')
                rclpy.shutdown()
                return

            if choice == 'c':
                self._custom_command()
                continue

            if choice == 'a':
                self._run_all_phases()
                continue

            # Numeric selection (1-5)
            try:
                idx = int(choice) - 1
                if 0 <= idx < len(SERVO_PHASES):
                    self._execute_phase(idx)
                else:
                    print(f'Invalid choice. Enter 1-{len(SERVO_PHASES)}, a, c, or q.')
            except ValueError:
                print(f'Invalid choice. Enter 1-{len(SERVO_PHASES)}, a, c, or q.')

    def _print_menu(self):
        ch = self.cfg['payload_servo_channel']
        print('\n' + '=' * 60)
        print('  DBVF SERVO TEST — Mission Phase Simulator')
        print('=' * 60)
        print(f'  Arduino service: /dbvf/arduino/set_servo')
        print(f'  Servo channel:   {ch}')
        print('-' * 60)
        for phase in SERVO_PHASES:
            _key, label, _ch_key, pwm_key, desc = phase
            pwm = self.cfg[pwm_key]
            print(f'  {label}')
            print(f'       PWM: {pwm}   ({desc})')
        print('-' * 60)
        print('  a) Run ALL phases in sequence (with pauses)')
        print('  c) Custom command (manual channel + PWM)')
        print('  q) Quit')

    def _execute_phase(self, idx):
        _key, label, ch_key, pwm_key, desc = SERVO_PHASES[idx]
        channel = self.cfg[ch_key]
        pwm = self.cfg[pwm_key]

        print(f'\n--- {label} ---')
        print(f'    {desc}')
        print(f'    Channel: {channel}, PWM: {pwm}')

        if pwm == 0:
            print('    WARNING: PWM is 0 (uncalibrated). Servo may not move.')
            try:
                override = input('    Enter PWM override (or press Enter to send 0): ').strip()
                if override:
                    pwm = int(override)
                    print(f'    Using override PWM: {pwm}')
            except (ValueError, EOFError, KeyboardInterrupt):
                pass

        try:
            confirm = input('    Send command? [Y/n]: ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            return
        if confirm in ('', 'y', 'yes'):
            self._call_servo(channel, pwm)
        else:
            print('    Skipped.')

    def _run_all_phases(self):
        """Step through all 5 phases in competition order with pauses."""
        import time
        print('\n=== Running ALL phases in competition order ===')
        print('Press Enter to advance to each phase, or q to stop.\n')

        for i, phase in enumerate(SERVO_PHASES):
            _key, label, ch_key, pwm_key, desc = phase
            channel = self.cfg[ch_key]
            pwm = self.cfg[pwm_key]

            print(f'\nNext: {label}')
            print(f'  Channel: {channel}, PWM: {pwm}')
            print(f'  {desc}')
            try:
                cmd = input('  [Enter]=send, [s]=skip, [q]=quit: ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                return

            if cmd == 'q':
                print('Stopped.')
                return
            if cmd == 's':
                print('  Skipped.')
                continue

            if pwm == 0:
                print('  WARNING: PWM is 0 (uncalibrated).')
                try:
                    override = input('  Enter PWM override (or Enter to send 0): ').strip()
                    if override:
                        pwm = int(override)
                except (ValueError, EOFError, KeyboardInterrupt):
                    pass

            self._call_servo(channel, pwm)

            # Settle time (matches mission_sequencer behavior)
            settle = 2.0
            print(f'  Settling for {settle}s...')
            time.sleep(settle)

        print('\n=== All phases complete ===')

    def _custom_command(self):
        """Send an arbitrary servo channel + PWM value."""
        try:
            ch = int(input('  Servo channel: ').strip())
            pwm = int(input('  PWM value: ').strip())
        except (ValueError, EOFError, KeyboardInterrupt):
            print('  Cancelled.')
            return
        self._call_servo(ch, pwm)


def main():
    rclpy.init()
    node = ServoTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
