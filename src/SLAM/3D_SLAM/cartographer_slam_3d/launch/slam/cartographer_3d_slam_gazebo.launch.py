"""
Cartographer 3D SLAM - Gazebo Simulation (Pioneer2dx)

3D LiDAR + IMU 기반 Cartographer 3D SLAM - Gazebo 시뮬레이션

Sensors (via Gazebo bridge):
- 3D LiDAR: /scan/points (PointCloud2, 16ch, 20Hz)
- IMU:       /imu/data   (Imu, 200Hz)

Prerequisite:
  ros2 launch tm_gazebo gazebo_no_odom.launch.py odom_tf:=false

Usage:
  ros2 launch cartographer_slam_3d cartographer_3d_slam_gazebo.launch.py

Map save (after mapping):
  ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
    "{filename: '$HOME/Study/ros2_3dslam_ws/maps/cartographer_3d/map.pbstream', include_unfinished_submaps: true}"
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_src = os.path.join(
        os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'src',
        'SLAM', '3D_SLAM', 'cartographer_slam_3d'
    )
    cartographer_config_dir = os.path.join(pkg_src, 'config')
    configuration_basename = 'cartographer_3d.lua'
    rviz_config_file = os.path.join(pkg_src, 'rviz', 'cartographer_3d.rviz')
    accumulator_script = os.path.join(pkg_src, 'scripts', 'map_cloud_accumulator.py')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock (Gazebo)'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename,
            ],
            remappings=[
                ('points2', '/scan/points'),  # 3D LiDAR PointCloud2
                ('imu', '/imu/data'),         # IMU direct
            ],
        ),

        # 맵 포인트클라우드 누적기: /scan_matched_points2 → /map_cloud
        ExecuteProcess(
            cmd=['python3', accumulator_script,
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen',
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
