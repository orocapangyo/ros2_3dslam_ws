"""
RTAB-Map Gazebo: 3D LiDAR + RGBD Localization
3D LiDAR (PointCloud2) + RGB-D Camera 위치 인식 - Gazebo 시뮬레이션

Sensor Combination / 센서 조합:
  - 3D LiDAR  : /scan/points  (PointCloud2, 16ch, 20Hz) → ICP Odometry
  - RGB-D Camera : /camera/color/image_raw + /camera/depth/image_raw → Visual Loop Closure

Requires an existing RTAB-Map database from the SLAM mapping phase.
SLAM 매핑 단계에서 생성된 RTAB-Map 데이터베이스(rtabmap.db)가 필요합니다.

Architecture:
  icp_odometry  →  rtabmap (localization, Reg/Strategy=2: ICP+Visual)
                →  point_cloud_xyzrgb + point_cloud_assembler (colored map visualization)

Required Topics (Gazebo bridge):
  /scan/points                    sensor_msgs/PointCloud2
  /camera/color/image_raw         sensor_msgs/Image
  /camera/color/camera_info       sensor_msgs/CameraInfo
  /camera/depth/image_raw         sensor_msgs/Image
  /camera/depth/camera_info       sensor_msgs/CameraInfo

TF (provided by Gazebo URDF):
  map → odom_rtabmap → base_footprint → lidar_link
                        base_footprint → camera_link

Usage:
  ros2 launch rtab_map_3d_config rtabmap_3dlidar_rgbd_localization_gazebo.launch.py
  ros2 launch rtab_map_3d_config rtabmap_3dlidar_rgbd_localization_gazebo.launch.py \\
    database_path:=$HOME/Study/ros2_3dslam_ws/maps/rtabmap.db
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_src = os.path.join(
        os.path.expanduser('~'),
        'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '3D_SLAM', 'rtab_map_3d'
    )
    rviz_config = os.path.join(pkg_src, 'rviz', 'rtabmap_localization.rviz')
    default_db_path = os.path.join(os.path.expanduser('~'), '.ros', 'rtabmap.db')

    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Launch RViz2')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use Gazebo simulation clock')
    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value=default_db_path,
        description='RTAB-Map database path for localization')

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

    # --- 2. RTAB-Map Localization (3D LiDAR + RGB-D) ---
    # Localization mode: DB에서 맵 로드, 새 노드 추가 안 함
    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[{'database_path': database_path}, {
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom_rtabmap',
            'map_frame_id': 'map',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_rgbd': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'queue_size': 10,
            # QoS: BEST_EFFORT (Gazebo bridge)
            'qos_image': 2,
            'qos_camera_info': 2,
            'qos_scan_cloud': 2,
            # Localization 모드 (SLAM과 핵심 차이)
            'Mem/IncrementalMemory': 'false',       # 새 노드 추가 안 함
            'Mem/InitWMWithAllNodes': 'true',        # DB 전체 노드 로드
            'RGBD/StartAtOrigin': 'true',            # 맵 즉시 표시
            # Loop closure / proximity
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            # Registration: ICP + Visual
            'Reg/Strategy': '2',
            'Reg/Force3DoF': 'false',
            # 3D Grid
            'Grid/RangeMax': '20.0',
            'Grid/RangeMin': '0.3',
            'Grid/CellSize': '0.05',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            # ICP (loop closure용)
            'Icp/VoxelSize': '0.1',
            'Icp/PointToPlane': 'true',
            # Visual features
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1',
            'Vis/FeatureType': '6',   # ORB
            'Vis/MaxFeatures': '1000',
            # Robust optimization (위치 인식 안정성)
            'Optimizer/Robust': 'true',
            'RGBD/OptimizeMaxError': '0.0',
        }],
        remappings=[
            ('scan_cloud', '/scan/points'),
            ('odom', '/rtabmap/odom'),
            ('odom_info', '/rtabmap/odom_info'),
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
        ],
        # '-d' 없음: 기존 DB 로드하여 위치 인식
        output='screen'
    )

    # --- 3. RGB-D 컬러 포인트 클라우드 생성 ---
    point_cloud_xyzrgb = Node(
        package='rtabmap_util',
        executable='point_cloud_xyzrgb',
        name='point_cloud_xyzrgb',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'decimation': 4,
            'max_depth': 4.0,
            'voxel_size': 0.02,
            'approx_sync': True,
            'queue_size': 10,
            # QoS: BEST_EFFORT (Gazebo bridge)
            'qos_image': 2,
            'qos_camera_info': 2,
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
        ],
        output='screen'
    )

    # --- 4. 컬러 포인트 클라우드 누적 ---
    point_cloud_assembler = Node(
        package='rtabmap_util',
        executable='point_cloud_assembler',
        name='rgb_cloud_assembler',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'fixed_frame_id': 'map',
            'max_clouds': 100,
            'circular_buffer': True,
            'voxel_size': 0.05,
            'wait_for_transform': 1.0,
        }],
        remappings=[('cloud', '/rtabmap/cloud')],
        output='screen'
    )

    # --- 5. RViz2 (지연 시작) ---
    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(rviz),
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        rviz_arg,
        use_sim_time_arg,
        database_path_arg,
        icp_odometry,
        rtabmap_localization,
        point_cloud_xyzrgb,
        point_cloud_assembler,
        rviz_node,
    ])
