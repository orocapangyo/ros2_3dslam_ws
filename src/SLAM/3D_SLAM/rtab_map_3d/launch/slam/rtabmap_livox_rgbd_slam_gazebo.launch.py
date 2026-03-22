"""
RTAB-Map 3D SLAM with Livox Mid-360 (3D LiDAR) + Astra Pro (RGB-D) - Gazebo Simulation
RTAB-Map 3D SLAM: Livox Mid-360 (3D LiDAR) + Astra Pro (RGB-D) 통합 런치 - Gazebo Simulation

Gazebo Simulation launch: ICP Odom + SLAM + RViz2
Gazebo 시뮬레이션 런치: ICP 오도메트리 + SLAM + RViz2
(Drivers and static TFs are provided by the Gazebo URDF/simulation environment)

Sensors / 센서:
- Livox Mid-360: 3D LiDAR (360° FOV, ~200k pts/frame) - simulated via Gazebo bridge
- Orbbec Astra Pro: RGB-D Camera - simulated via Gazebo bridge

Published Topics (from Gazebo bridge):
- /livox/lidar (sensor_msgs/PointCloud2)
- /camera/color/image_raw (sensor_msgs/Image)
- /camera/color/camera_info (sensor_msgs/CameraInfo)
- /camera/depth/image_raw (sensor_msgs/Image)
- /camera/depth/camera_info (sensor_msgs/CameraInfo)

TF Tree (provided by Gazebo URDF):
  base_link -> livox_frame
  base_link -> camera_link -> camera_color_frame -> camera_color_optical_frame
                              camera_depth_frame -> camera_depth_optical_frame

Usage / 사용법:
  ros2 launch rtab_map_3d rtabmap_livox_rgbd_slam_gazebo.launch.py
  ros2 launch rtab_map_3d rtabmap_livox_rgbd_slam_gazebo.launch.py rviz:=false
  ros2 launch rtab_map_3d rtabmap_livox_rgbd_slam_gazebo.launch.py use_sim_time:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Livox Mid-360 + Astra Pro 3D SLAM (Gazebo Simulation)."""

    # 패키지 소스 경로 (src 폴더 규약)
    pkg_src = os.path.join(
        os.path.expanduser('~'),
        'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '3D_SLAM', 'rtab_map_3d'
    )

    rviz_config = os.path.join(pkg_src, 'rviz2', 'rtabmap_livox_rgbd_slam.rviz')

    # Launch arguments
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

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

    # --- 1. ICP Odometry (Livox 3D 포인트 클라우드 기반) ---
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
            # ICP 파라미터 (Livox Mid-360 튜닝, 360° FOV, ~20k pts/frame)
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
        remappings=[
            ('scan_cloud', '/scan/points'),
        ],
        output='screen'
    )

    # --- 2. RTAB-Map SLAM (3D LiDAR + RGB-D 융합) ---
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
            # RTAB-Map 파라미터 (SLAM 모드)
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            # 루프 클로저 / 근접 탐지
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            # Registration: ICP + Visual 결합 (LiDAR + 카메라 동시 사용)
            'Reg/Strategy': '2',
            'Reg/Force3DoF': 'false',
            # 3D Grid/Cloud 파라미터
            'Grid/RangeMax': '20.0',
            'Grid/RangeMin': '0.3',
            'Grid/CellSize': '0.05',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            # ICP 파라미터 (루프 클로저용)
            'Icp/VoxelSize': '0.1',
            'Icp/PointToPlane': 'true',
            # 시각적 특징 파라미터
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1',
            'Vis/FeatureType': '6',     # ORB
            'Vis/MaxFeatures': '1000',
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
        arguments=['-d'],  # Delete database on start (fresh mapping)
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
        rviz_arg,
        use_sim_time_arg,
        icp_odometry,
        rtabmap_slam,
        point_cloud_xyzrgb,
        rgb_cloud_assembler,
        rviz_node,
    ])
