"""
RTAB-Map Gazebo 2D LiDAR SLAM

가제보 시뮬레이션용. LiDAR ICP 기반 SLAM.

Required Topics:
- /scan (sensor_msgs/LaserScan)

Usage:
    ros2 launch rtab_map_config rtabmap_lidar_slam.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_src = os.path.join(
        os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '2D_SLAM', 'rtab_map'
    )
    rviz_config = os.path.join(pkg_src, 'rviz', 'rtabmap_lidar.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    rviz = LaunchConfiguration('rviz')

    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': False,  # Gazebo가 odom TF 제공
            'wait_for_transform': 0.2,
            'topic_queue_size': 10,
            'Icp/VoxelSize': '0.05',
            'Icp/MaxCorrespondenceDistance': '0.1',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlane': 'false',
            'Odom/ScanKeyFrameThr': '0.6',
            'OdomF2M/ScanSubtractRadius': '0.05',
            'OdomF2M/ScanMaxSize': '5000',
            # QoS: BEST_EFFORT to match Gazebo bridge output
            'qos_scan': 2,
        }],
        remappings=[('scan', scan_topic)],
        output='screen'
    )

    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_rgbd': False,
            'subscribe_scan': True,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'queue_size': 10,
            # QoS: BEST_EFFORT to match Gazebo bridge output
            'qos_scan': 2,
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'true',
            'Grid/RangeMax': '5.0',
        }],
        remappings=[
            ('scan', scan_topic),
            ('odom', '/rtabmap/odom'),
        ],
        arguments=['-d'],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation time (Gazebo 기본값 true)'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='Laser scan topic'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz'),
        icp_odometry,
        rtabmap_slam,
        rviz_node,
    ])
