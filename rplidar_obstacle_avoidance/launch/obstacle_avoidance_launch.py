#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments for easy parameter configuration
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for RPLIDAR'
        ),
        
        DeclareLaunchArgument(
            'min_distance',
            default_value='1.0',
            description='Minimum distance threshold in meters'
        ),
        
        DeclareLaunchArgument(
            'fov_start',
            default_value='0.0',
            description='FOV start angle in degrees'
        ),
        
        DeclareLaunchArgument(
            'fov_end',
            default_value='120.0',
            description='FOV end angle in degrees'
        ),
        
        # RPLIDAR node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': 1000000,
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            output='screen'
        ),
        
        # Obstacle avoidance node
        # Config file is loaded first, then launch arguments override it
        Node(
            package='rplidar_obstacle_avoidance',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance_node',
            parameters=[
                os.path.join(
                    get_package_share_directory('rplidar_obstacle_avoidance'),
                    'config',
                    'obstacle_avoidance.yaml'
                ),
                {
                    # Override with launch arguments if provided
                    'min_distance_threshold': LaunchConfiguration('min_distance'),
                    'fov_start_angle': LaunchConfiguration('fov_start'),
                    'fov_end_angle': LaunchConfiguration('fov_end'),
                }
            ],
            output='screen'
        ),
    ])
