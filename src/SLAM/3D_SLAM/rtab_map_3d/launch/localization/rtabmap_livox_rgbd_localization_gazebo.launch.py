"""
RTAB-Map 3D Localization with Livox Mid-360 (3D LiDAR) + Astra Pro (RGB-D) - Gazebo Simulation
Livox Mid-360 + Astra Pro 기반 RTAB-Map 3D 위치 인식 - Gazebo Simulation

Gazebo Simulation launch: ICP Odom + Localization + RViz2
Gazebo 시뮬레이션 런치: ICP 오도메트리 + 위치 인식 + RViz2
(Drivers and static TFs are provided by the Gazebo URDF/simulation environment)

Requires existing RTAB-Map database from SLAM mapping phase.
SLAM 매핑 단계에서 생성된 RTAB-Map 데이터베이스가 필요합니다.

Usage / 사용법:
  ros2 launch rtab_map_3d rtabmap_livox_rgbd_localization_gazebo.launch.py
  ros2 launch rtab_map_3d rtabmap_livox_rgbd_localization_gazebo.launch.py database_path:=/path/to/rtabmap.db

Sensors (via Gazebo bridge):
- Livox Mid-360 (3D LiDAR) -> /livox/lidar (PointCloud2)
- Orbbec Astra Pro (RGB-D)  -> /camera/color/image_raw, /camera/depth/image_raw

TF Tree (provided by Gazebo URDF):
  map -> odom -> base_link -> livox_frame
                 base_link -> camera_link -> camera_color_frame -> camera_color_optical_frame
                                             camera_depth_frame -> camera_depth_optical_frame
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Livox Mid-360 + Astra Pro 3D Localization (Gazebo Simulation)."""

    # 패키지 소스 경로 (src 폴더 규약)
    pkg_src = os.path.join(
        os.path.expanduser('~'),
        'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '3D_SLAM', 'rtab_map_3d'
    )

    rviz_config = os.path.join(pkg_src, 'rviz2', 'rtabmap_livox_rgbd_localization.rviz')

    # Launch arguments
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    # 기본 DB 경로: os.path.expanduser로 tilde 확장 (ROS2 LaunchConfig은 ~ 미확장)
    default_db_path = os.path.join(os.path.expanduser('~'), '.ros', 'rtabmap.db')

    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value=default_db_path,
        description='RTAB-Map database path for localization'
    )

    # --- 1. ICP Odometry (3D LiDAR용) ---
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
            'wait_for_transform': 1.0,
            'approx_sync': True,
            'queue_size': 10,
            # Odometry 리셋: lost 상태 시 3프레임 후 자동 리셋
            'Odom/ResetCountdown': '3',
            # ICP 파라미터 (Livox Mid-360 튜닝, ~20k pts/frame)
            'Icp/VoxelSize': '0.2',
            'Icp/MaxCorrespondenceDistance': '1.0',
            'Icp/RangeMax': '20.0',
            'Icp/RangeMin': '0.3',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlane': 'false',
            'Icp/PointToPlaneK': '10',
            'Icp/PointToPlaneRadius': '0.0',
            'Odom/ScanKeyFrameThr': '0.6',
            'OdomF2M/ScanSubtractRadius': '0.2',
            'OdomF2M/ScanMaxSize': '15000',
            # QoS: BEST_EFFORT to match Gazebo bridge output
            'qos_scan_cloud': 2,
        }],
        remappings=[('scan_cloud', '/scan/points')],
        output='screen'
    )

    # --- 2. RTAB-Map Localization 노드 (3D LiDAR + RGB-D) ---
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
            # 센서 구독 설정
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_rgbd': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'queue_size': 10,
            # QoS: BEST_EFFORT to match Gazebo bridge output
            'qos_image': 2,
            'qos_camera_info': 2,
            'qos_scan_cloud': 2,
            # === Localization 모드 (SLAM과 핵심 차이) ===
            'Mem/IncrementalMemory': 'false',       # 새 노드 추가하지 않음
            'Mem/InitWMWithAllNodes': 'true',        # DB에서 모든 노드 로드
            'RGBD/StartAtOrigin': 'true',            # 맵 즉시 표시
            # Loop closure / proximity
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            # Registration: ICP + Visual 결합
            'Reg/Strategy': '2',
            'Reg/Force3DoF': 'false',
            # 3D Grid/Cloud 파라미터
            'Grid/RangeMax': '20.0',
            'Grid/RangeMin': '0.3',
            'Grid/CellSize': '0.05',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            # ICP 파라미터 (loop closure용)
            'Icp/VoxelSize': '0.1',
            'Icp/PointToPlane': 'true',
            # 시각적 특징 파라미터
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1',
            'Vis/FeatureType': '6',     # ORB
            'Vis/MaxFeatures': '1000',
            # Robust optimization (위치 인식용)
            'Optimizer/Robust': 'true',
            'RGBD/OptimizeMaxError': '0.0',  # Robust optimizer 사용 시 비활성화
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
        # '-d' 없음! 기존 데이터베이스를 로드하여 위치 인식 수행
        output='screen'
    )

    # --- 3. RGB-D 컬러 포인트 클라우드 생성 (카메라 RGB 텍스처 3D 맵) ---
    # depth + RGB → per-frame XYZRGB 포인트 클라우드
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
            # QoS: BEST_EFFORT to match Gazebo bridge output
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

    # --- 4. RGB 포인트 클라우드 누적 (3D RGB 맵 조립) ---
    # per-frame XYZRGB cloud → 누적하여 컬러 3D 맵 생성
    rgb_cloud_assembler = Node(
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
        remappings=[
            ('cloud', '/rtabmap/cloud'),
        ],
        output='screen'
    )

    # --- 5. RViz2 (지연 시작) ---
    # cloud_map 토픽 데이터가 준비된 후 RViz2가 로드되어야
    # Color Transformer: RGB8 설정이 정상 적용됨
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
        # Launch arguments
        rviz_arg,
        use_sim_time_arg,
        database_path_arg,
        # Components
        icp_odometry,
        rtabmap_localization,
        point_cloud_xyzrgb,
        rgb_cloud_assembler,
        rviz_node,
    ])
