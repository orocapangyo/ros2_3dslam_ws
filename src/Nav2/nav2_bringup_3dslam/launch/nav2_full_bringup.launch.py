"""
Full-stack bringup: Gazebo + RTAB-Map 3D LiDAR localization + Nav2.

Single command that brings up:
  1. Gazebo (Ignition) with pioneer2dx, odom_tf:=false (RTAB-Map provides
     the odom->base_footprint TF via icp_odometry)
  2. RTAB-Map 3D LiDAR-only localization (loads existing DB from
     maps/rtabmap_3d/rtabmap_3dlidar_only.db by default)
  3. Nav2 smac_hybrid stack (Smac Hybrid-A* planner + RPP controller)

The user must stop AMR motion control separately before running this, since
AMR publishes on /cmd_vel and competes with Nav2's velocity_smoother output:

  pkill -f amr_motion_control_2wd

Usage:
  ros2 launch nav2_bringup_3dslam nav2_full_bringup.launch.py
  ros2 launch nav2_bringup_3dslam nav2_full_bringup.launch.py \
    database_path:=/absolute/path/to/rtabmap.db

Verified 2026-04-19 on feat/nav2-goal-reach: NavigateToPose (2.0, 0.0)
reached with error < 0.25m.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav2 = get_package_share_directory('nav2_smac_hybrid')
    pkg_gazebo = get_package_share_directory('tm_gazebo')
    pkg_rtabmap = get_package_share_directory('rtab_map_3d_config')
    pkg_bringup = get_package_share_directory('nav2_bringup_3dslam')

    default_db = os.path.join(
        os.path.expanduser('~'),
        'Study', 'ros2_3dslam_ws', 'maps', 'rtabmap_3d',
        'rtabmap_3dlidar_only.db',
    )
    default_params = os.path.join(pkg_nav2, 'config', 'smac_hybrid_params.yaml')
    default_rviz = os.path.join(pkg_bringup, 'rviz2', 'nav2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    database_path = LaunchConfiguration('database_path')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('database_path', default_value=default_db),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch Nav2 RViz2 with /plan path display'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'odom_tf': 'false'}.items(),
        ),

        # RTAB-Map localization — disable its own RViz; Nav2 RViz below replaces it
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    pkg_rtabmap, 'launch', 'localization',
                    'rtabmap_3dlidar_only_localization_gazebo.launch.py',
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rviz': 'false',
                'database_path': database_path,
            }.items(),
        ),

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

        # Nav2 RViz with /plan, /plan_smoothed, /received_global_plan path displays
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
