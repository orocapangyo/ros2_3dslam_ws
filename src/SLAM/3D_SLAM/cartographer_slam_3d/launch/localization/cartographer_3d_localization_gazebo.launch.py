"""
Cartographer 3D Localization - Gazebo Simulation (Pioneer2dx)

기존 .pbstream 3D 지도 파일 기반 위치 추정 - Gazebo 시뮬레이션

Sensors (via Gazebo bridge):
- 3D LiDAR: /scan/points (PointCloud2, 16ch, 20Hz)
- IMU:       /imu/data   (Imu, 200Hz)

Prerequisite:
  ros2 launch tm_gazebo gazebo_no_odom.launch.py odom_tf:=false

Usage:
  ros2 launch cartographer_slam_3d cartographer_3d_localization_gazebo.launch.py
  ros2 launch cartographer_slam_3d cartographer_3d_localization_gazebo.launch.py \
    map_file:=/path/to/map.pbstream
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_src = os.path.join(
        os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'src',
        'SLAM', '3D_SLAM', 'cartographer_slam_3d'
    )
    cartographer_config_dir = os.path.join(pkg_src, 'config')
    configuration_basename = 'cartographer_3d_localization.lua'
    rviz_config_file = os.path.join(pkg_src, 'rviz', 'cartographer_3d.rviz')

    default_map_file = os.path.join(
        os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'maps',
        'cartographer_3d', 'map.pbstream'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock (Gazebo)'),
        DeclareLaunchArgument(
            'map_file',
            default_value=default_map_file,
            description='Path to .pbstream 3D map file'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename,
                '-load_state_filename', map_file,
            ],
            remappings=[
                ('points2', '/scan/points'),  # 3D LiDAR PointCloud2
                ('imu', '/imu/data'),         # IMU direct
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
