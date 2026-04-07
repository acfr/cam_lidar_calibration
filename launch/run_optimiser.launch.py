#!/usr/bin/env python3
"""
Launch file for camera-lidar calibration optimizer.

Usage:
  ros2 launch cam_lidar_calibration run_optimiser.launch.py import_samples:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for camera-lidar calibration."""
    
    # Declare launch arguments
    import_samples_arg = DeclareLaunchArgument(
        'import_samples',
        default_value='false',
        description='Whether to import samples from CSV file'
    )
    
    # Package paths
    pkg_share = FindPackageShare('cam_lidar_calibration')
    params_file = PathJoinSubstitution([pkg_share, 'cfg', 'params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'cam_lidar_calibration.rviz'])
    import_path = PathJoinSubstitution([pkg_share, 'data', 'vlp', 'poses.csv'])
    
    # Feature extraction node
    feature_extraction_node = Node(
        package='cam_lidar_calibration',
        executable='feature_extraction_node',
        name='feature_extraction',
        output='screen',
        parameters=[
            params_file,
            {
                'num_lowestvoq': 50,
                'import_samples': LaunchConfiguration('import_samples'),
                'import_path': import_path,
                'distance_offset_mm': 0.0,
            }
        ]
    )
    
    # RViz node (only if not importing samples)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=UnlessCondition(LaunchConfiguration('import_samples'))
    )
    
    # Note: rqt_reconfigure equivalent in ROS2 is ros2 param or rqt
    # You can use: ros2 run rqt_reconfigure rqt_reconfigure
    
    rmw_zenoh_node = Node(
        package='rmw_zenoh_cpp',
        executable='rmw_zenoh_cpp',
        name='rmw_zenoh_cpp',
        output='screen',
    )
    
    return LaunchDescription([
        rmw_zenoh_node,
        import_samples_arg,
        feature_extraction_node,
        rviz_node,
    ])
