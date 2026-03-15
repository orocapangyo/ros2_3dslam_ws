"""
RTAB-Map Gazebo: 3D LiDAR Only SLAM
3D LiDAR (PointCloud2) 단독 SLAM - Gazebo 시뮬레이션 (카메라 없음)

Sensor Combination / 센서 조합:
  - 3D LiDAR  : /scan/points  (PointCloud2, 16ch, 20Hz) → ICP Odometry + SLAM

Architecture:
  icp_odometry  →  rtabmap (Reg/Strategy=1: ICP only)

Required Topics (Gazebo bridge):
  /scan/points    sensor_msgs/PointCloud2

TF (provided by Gazebo URDF):
  map → odom_rtabmap → base_footprint → lidar_link

Usage:
  ros2 launch rtab_map_3d_config rtabmap_3dlidar_only_slam_gazebo.launch.py
  ros2 launch rtab_map_3d_config rtabmap_3dlidar_only_slam_gazebo.launch.py rviz:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_src = os.path.join(
        os.path.expanduser('~'),
        'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '3D_SLAM', 'rtab_map_3d'
    )
    rviz_config = os.path.join(pkg_src, 'rviz', 'rtabmap_3d.rviz')

    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Launch RViz2')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use Gazebo simulation clock')

    # --- 1. ICP Odometry (3D LiDAR 기반) ---
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom_rtabmap',
            'publish_tf': True,
            'wait_for_transform': 0.2,
            'approx_sync': True,
            'queue_size': 10,
            'Odom/ResetCountdown': '3',
            'Icp/VoxelSize': '0.1',
            'Icp/MaxCorrespondenceDistance': '0.5',
            'Icp/RangeMax': '20.0',
            'Icp/RangeMin': '0.3',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0.0',
            'Odom/ScanKeyFrameThr': '0.6',
            'OdomF2M/ScanSubtractRadius': '0.1',
            'OdomF2M/ScanMaxSize': '15000',
            # QoS: BEST_EFFORT (Gazebo bridge)
            'qos_scan_cloud': 2,
        }],
        remappings=[('scan_cloud', '/scan/points')],
        output='screen'
    )

    # --- 2. RTAB-Map SLAM (3D LiDAR 전용) ---
    # Reg/Strategy=1: ICP 스캔 정합만 사용 (카메라 없음)
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom_rtabmap',
            'map_frame_id': 'map',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_rgbd': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'queue_size': 10,
            # QoS: BEST_EFFORT (Gazebo bridge)
            'qos_scan_cloud': 2,
            # SLAM 모드
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            # Loop closure / proximity (LiDAR 스캔 기반)
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            # Registration: ICP only
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'false',
            # 3D Grid
            'Grid/RangeMax': '20.0',
            'Grid/RangeMin': '0.3',
            'Grid/CellSize': '0.1',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            # ICP (loop closure용)
            'Icp/VoxelSize': '0.1',
            'Icp/PointToPlane': 'true',
        }],
        remappings=[
            ('scan_cloud', '/scan/points'),
            ('odom', '/rtabmap/odom'),
            ('odom_info', '/rtabmap/odom_info'),
        ],
        arguments=['-d'],
        output='screen'
    )

    # --- 3. RViz2 ---
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
        rviz_arg,
        use_sim_time_arg,
        icp_odometry,
        rtabmap_slam,
        rviz_node,
    ])
