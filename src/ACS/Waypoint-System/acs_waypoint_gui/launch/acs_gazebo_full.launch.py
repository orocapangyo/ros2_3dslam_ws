"""ACS Full Gazebo Launch — Gazebo + Motion Control + Waypoint Manager + ACS GUI.

Usage:
  ros2 launch acs_waypoint_gui acs_gazebo_full.launch.py
  ros2 launch acs_waypoint_gui acs_gazebo_full.launch.py use_slam:=true
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    RegisterEventHandler, EmitEvent,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ws_src = os.path.join(os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'src')
    use_slam = LaunchConfiguration('use_slam')

    # --- Layer 0: Gazebo ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ws_src, 'Gazebo', 'launch', 'gazebo.launch.py')),
    )

    # --- Static TF: map -> odom (only when NOT using SLAM) ---
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=UnlessCondition(use_slam),
    )

    # --- Layer 2: Motion Control ---
    motion_config = os.path.join(
        get_package_share_directory('amr_motion_control_2wd'),
        'config', 'motion_params_gazebo_no_slam.yaml')

    motion_control = Node(
        package='amr_motion_control_2wd',
        executable='amr_motion_control_2wd_node',
        name='amr_motion_control_2wd',
        parameters=[motion_config, {'use_sim_time': True}],
        output='screen',
    )

    # --- Layer 3: Waypoint Manager ---
    wp_config = os.path.join(
        get_package_share_directory('waypoint_manager'),
        'config', 'waypoint_params_gazebo.yaml')

    waypoint_mgr = Node(
        package='waypoint_manager',
        executable='waypoint_manager_node',
        name='waypoint_manager',
        parameters=[wp_config, {'use_sim_time': True}],
        output='screen',
    )

    # --- Layer 4: ACS GUI ---
    acs_gui = Node(
        package='acs_waypoint_gui',
        executable='acs_gui_node',
        name='acs_gui_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # GUI exit → shutdown all
    shutdown_on_gui_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=acs_gui,
            on_exit=[EmitEvent(event=Shutdown(reason='ACS GUI closed'))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_slam', default_value='false',
                              description='Set true when SLAM provides map->odom TF'),

        gazebo_launch,
        static_tf_map_odom,
        motion_control,
        waypoint_mgr,
        acs_gui,
        shutdown_on_gui_exit,
    ])
