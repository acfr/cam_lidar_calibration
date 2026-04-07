#!/usr/bin/env python3
"""
Launch file for assessing calibration results.

Usage:
  ros2 launch cam_lidar_calibration assess_results.launch.py csv:="/path/to/results.csv"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for assessing calibration results."""
    
    # Declare launch arguments
    csv_arg = DeclareLaunchArgument(
        'csv',
        default_value='',
        description='Path to the calibration results CSV file'
    )
    
    visualise_arg = DeclareLaunchArgument(
        'visualise',
        default_value='true',
        description='Whether to visualize the results'
    )
    
    # Package paths
    pkg_share = FindPackageShare('cam_lidar_calibration')
    params_file = PathJoinSubstitution([pkg_share, 'cfg', 'params.yaml'])
    camera_info_file = PathJoinSubstitution([pkg_share, 'cfg', 'camera_info.yaml'])
    
    # Visualize results node (Python script)
    visualise_results_node = Node(
        package='cam_lidar_calibration',
        executable='visualise_results.py',
        name='visualise_results',
        output='screen',
        parameters=[
            params_file,
            camera_info_file,
            {
                'use_sim_time': False,
                'csv': LaunchConfiguration('csv'),
                'degree': False,
                'trans_binwidth': 0.01,
                'rot_binwidth_deg': 0.5,
            }
        ]
    )
    
    # Assessment node
    assess_node = Node(
        package='cam_lidar_calibration',
        executable='assess_node',
        name='assess',
        output='screen',
        parameters=[
            params_file,
            camera_info_file,
            {
                'use_sim_time': False,
                'csv': LaunchConfiguration('csv'),
                'visualise': LaunchConfiguration('visualise'),
                'visualise_pose_num': 3,
            }
        ]
    )

    rmw_zenoh_node = Node(
        package='rmw_zenoh_cpp',
        executable='rmw_zenoh_cpp',
        name='rmw_zenoh_cpp',
        output='screen',
    )
    
    return LaunchDescription([
        csv_arg,
        visualise_arg,
        rmw_zenoh_node,
        visualise_results_node,
        assess_node,
    ])
