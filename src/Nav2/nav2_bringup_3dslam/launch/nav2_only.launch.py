"""
Nav2-only bringup.

Assumes Gazebo + RTAB-Map 3D localization are already running and publishing:
  - /rtabmap/map    (nav_msgs/OccupancyGrid, RELIABLE, TRANSIENT_LOCAL)
  - /scan           (sensor_msgs/LaserScan)
  - TF: map -> odom_rtabmap -> base_footprint -> base_link -> lidar_link
  - /cmd_vel        (Gazebo diff_drive subscribes)

Usage:
  # prerequisite (separate terminals)
  ros2 launch tm_gazebo gazebo.launch.py odom_tf:=false
  ros2 launch rtab_map_3d_config rtabmap_3dlidar_only_localization_gazebo.launch.py

  # stop AMR motion control first (it competes for /cmd_vel)
  pkill -f amr_motion_control_2wd

  # then
  ros2 launch nav2_bringup_3dslam nav2_only.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav2 = get_package_share_directory('nav2_smac_hybrid')
    pkg_bringup = get_package_share_directory('nav2_bringup_3dslam')
    default_params = os.path.join(pkg_nav2, 'config', 'smac_hybrid_params.yaml')
    default_rviz = os.path.join(pkg_bringup, 'rviz2', 'nav2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch Nav2 RViz2 with /plan path display'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'smac_hybrid_planner.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_nav2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(rviz),
        ),
    ])
