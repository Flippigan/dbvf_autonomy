"""Launch full mission stack for Gazebo simulation.

Prerequisites: iris_runway.launch.py must be running separately.
Includes all nodes from precision_landing_sim.launch.py plus the mission sequencer.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('dbvf_autonomy')
    sim_config = os.path.join(pkg_dir, 'config', 'sim_params.yaml')
    mission_config = os.path.join(pkg_dir, 'config', 'mission_params.yaml')

    return LaunchDescription([
        # AprilTag detector
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            remappings=[
                ('image_rect', '/camera/image'),
                ('camera_info', '/camera/camera_info'),
                ('detections', '/apriltag/detections'),
            ],
            parameters=[{
                'family': '36h11',
                'size': 0.15,
                'tag.ids': [1, 2],
                'tag.sizes': [0.15, 0.10],
            }],
        ),

        # Tag detector adapter
        Node(
            package='dbvf_autonomy',
            executable='tag_detector_adapter_node',
            name='tag_detector_adapter',
            parameters=[sim_config],
        ),

        # MAVLink interface
        Node(
            package='dbvf_autonomy',
            executable='mavlink_interface_node',
            name='mavlink_interface',
            parameters=[sim_config],
        ),

        # Precision landing state machine
        Node(
            package='dbvf_autonomy',
            executable='precision_landing_node',
            name='precision_landing',
            parameters=[sim_config],
        ),

        # Tag detection visualizer
        Node(
            package='dbvf_autonomy',
            executable='tag_visualizer_node',
            name='tag_visualizer',
        ),

        # Arduino serial interface
        Node(
            package='dbvf_autonomy',
            executable='arduino_interface_node',
            name='arduino_interface',
            parameters=[sim_config],
        ),

        # Mission sequencer
        Node(
            package='dbvf_autonomy',
            executable='mission_sequencer_node',
            name='mission_sequencer',
            parameters=[mission_config],
        ),
    ])
