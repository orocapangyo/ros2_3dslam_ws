"""
Cartographer Gazebo Localization Launch File

가제보 시뮬레이션용. 기존 .pbstream 지도 파일을 로드하여 위치추정만 수행.

Usage:
    ros2 launch cartographer_slam cartographer_localization.launch.py
    ros2 launch cartographer_slam cartographer_localization.launch.py map_file:=/path/to/map.pbstream
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_src = os.path.join(
        os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '2D_SLAM', 'Cartographer'
    )
    cartographer_config_dir = os.path.join(pkg_src, 'config')
    configuration_basename = 'cartographer_localization.lua'
    rviz_config_file = os.path.join(pkg_src, 'rviz', 'cartographer.rviz')

    default_map_file = os.path.join(
        os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'maps', 'cartographer', 'map.pbstream'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    map_file = LaunchConfiguration('map_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock (Gazebo 기본값 true)'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='Laser scan topic'),
        DeclareLaunchArgument('map_file', default_value=default_map_file,
                              description='Path to .pbstream map file'),

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
                ('scan', scan_topic),
                ('odom', '/odom'),
            ],
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05'],
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
