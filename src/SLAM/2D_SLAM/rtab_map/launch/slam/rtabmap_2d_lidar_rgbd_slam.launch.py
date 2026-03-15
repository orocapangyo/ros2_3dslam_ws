"""
RTAB-Map Gazebo 2D LiDAR (주) + RGB-D (보조) SLAM / 가제보 시뮬레이션용

2D LiDAR ICP 기반 odometry/registration + RGB-D 카메라 보조 (3D 포인트클라우드 색상, Grid 보조).
가제보 시뮬레이션 전용 버전 (use_sim_time=true, RELIABLE QoS, base_link, publish_tf=False).

Required Topics (Gazebo):
- /scan (sensor_msgs/LaserScan)
- /camera/color/image_raw (sensor_msgs/Image)
- /camera/color/camera_info (sensor_msgs/CameraInfo)
- /camera/depth/image_raw (sensor_msgs/Image)
- /camera/depth/camera_info (sensor_msgs/CameraInfo)

Usage:
    ros2 launch rtab_map_config rtabmap_2d_lidar_rgbd_slam.launch.py
    ros2 launch rtab_map_config rtabmap_2d_lidar_rgbd_slam.launch.py scan_topic:=/scan
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
        'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '2D_SLAM', 'rtab_map'
    )
    rviz_config = os.path.join(pkg_src, 'rviz', 'rtabmap_2d_lidar_rgbd.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    rviz = LaunchConfiguration('rviz')

    # --- 1. ICP Odometry (LiDAR 기반) ---
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': False,
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
        remappings=[
            ('scan', scan_topic),
        ],
        output='screen'
    )

    # --- 2. RTABMAP SLAM (2D LiDAR 주 + RGB-D 보조) ---
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
            # 센서 구독 설정
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_rgbd': False,
            'subscribe_scan': True,
            'subscribe_scan_cloud': False,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'approx_sync_max_interval': 0.5,
            'queue_size': 30,
            # QoS: BEST_EFFORT to match Gazebo bridge output
            'qos_image': 2,
            'qos_camera_info': 2,
            'qos_scan': 2,
            # RTAB-Map 파라미터
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            # Loop closure / proximity
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            # Registration: ICP (LiDAR 주)
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'true',
            # Grid/Map 파라미터
            'Grid/Sensor': '2',
            'Grid/RangeMax': '10.0',
            'Grid/RangeMin': '0.3',
            'Grid/CellSize': '0.05',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            # 시각적 특징 파라미터
            'Vis/MinInliers': '15',
            'Vis/FeatureType': '6',
            'Vis/MaxFeatures': '1000',
        }],
        remappings=[
            ('scan', scan_topic),
            ('odom', '/rtabmap/odom'),
            ('odom_info', '/rtabmap/odom_info'),
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
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
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time (Gazebo)'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Laser scan topic (default: /scan for Gazebo)'
        ),
        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            description='Use RGB-D camera as auxiliary sensor (Gazebo: always true)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),
        icp_odometry,
        rtabmap_slam,
        rviz_node,
    ])
