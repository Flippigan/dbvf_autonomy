"""Launch Arduino interface for servo testing (Jetson Orin Nano).

Starts only the Arduino serial interface node. Run the interactive
servo test node in a separate terminal.

Usage:
    # Terminal 1: Start Arduino interface
    ros2 launch dbvf_autonomy servo_test.launch.py

    # Terminal 2: Run interactive servo test
    ros2 run dbvf_autonomy servo_test_node --ros-args \
        --params-file install/dbvf_autonomy/share/dbvf_autonomy/config/mission_params.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('dbvf_autonomy')
    hw_config = os.path.join(pkg_dir, 'config', 'hardware_params.yaml')

    return LaunchDescription([
        # Arduino serial interface (owns /dev/ttyACM0)
        Node(
            package='dbvf_autonomy',
            executable='arduino_interface_node',
            name='arduino_interface',
            parameters=[hw_config],
            output='screen',
        ),
    ])
