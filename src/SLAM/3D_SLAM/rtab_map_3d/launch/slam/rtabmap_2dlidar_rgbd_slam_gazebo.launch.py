"""
RTAB-Map Gazebo 2D LiDAR + RGB-D Mapping Launch File

This launch file is for SLAM using both 2D LiDAR and RGB-D camera in Gazebo.
Based on rtabmap_gazebo_rgbd.launch.py with LiDAR support added.

Required Topics:
- /camera/color/image_raw (sensor_msgs/Image)
- /camera/color/camera_info (sensor_msgs/CameraInfo)
- /camera/depth/image_raw (sensor_msgs/Image)
- /scan (sensor_msgs/LaserScan)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for 2D LiDAR + RGB-D SLAM."""

    # src folder path (CLAUDE.md rule)
    pkg_src = os.path.join(
        os.path.expanduser('~'),
        'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '3D_SLAM', 'rtab_map_3d'
    )
    rviz_config = os.path.join(pkg_src, 'rviz2', 'rtabmap_lidar_rgbd.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    rviz = LaunchConfiguration('rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (default true for Gazebo)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='2D LiDAR scan topic'
    )

    # RGBD Odometry node (same as rtabmap_gazebo_rgbd.launch.py)
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'publish_tf': True,  # Use visual odometry TF (disable Gazebo's odom_to_tf)
            'wait_for_transform': 0.2,
            'approx_sync': True,
            'queue_size': 10,
            # QoS: BEST_EFFORT to match Gazebo bridge output
            'qos_image': 2,
            'qos_camera_info': 2,
            # Visual odometry parameters
            'Odom/Strategy': '0',
            'Odom/ResetCountdown': '1',
            'Vis/FeatureType': '6',  # ORB
            'Vis/CorType': '0',
            'Vis/MaxFeatures': '1000',
            'Vis/MinInliers': '15',
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
        ],
        output='screen'
    )

    # RTABMAP SLAM node (RGB-D + 2D LiDAR)
    # Based on rtabmap_gazebo_rgbd.launch.py with LiDAR added
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            # Sensor subscription settings
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_rgbd': False,
            'subscribe_scan': True,  # 2D LiDAR enabled
            'subscribe_scan_cloud': False,
            'subscribe_odom_info': True,  # Re-enabled: sync RGB+Depth+OdomInfo+Scan
            'approx_sync': True,
            'approx_sync_max_interval': 0.5,  # 500ms for RGB+Depth+Scan sync
            'queue_size': 30,
            # QoS: Gazebo bridge publishes BEST_EFFORT → set nodes to match
            'qos_image': 2,
            'qos_camera_info': 2,
            'qos_scan': 2,
            # RTAB-Map parameters
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            # Loop closure detection
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            # Registration (Visual only - scan used for 2D grid only)
            'Reg/Strategy': '0',  # 0=Visual (scan for grid, not registration)
            'Reg/Force3DoF': 'false',
            # Grid/Map parameters
            'Grid/Sensor': '1',  # CRITICAL: Force point cloud projection (0=scan only, 1=cloud, default auto)
            'Grid/RangeMax': '10.0',  # Extended for LiDAR (CHANGED from 5.0)
            'Grid/RangeMin': '0.3',
            'Grid/CellSize': '0.05',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            # Visual feature parameters
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1',
            'Vis/FeatureType': '6',  # ORB
            'Vis/MaxFeatures': '1000',
        }],
        remappings=[
            ('odom', '/rtabmap/odom'),
            ('odom_info', '/rtabmap/odom_info'),  # From rgbd_odometry
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
            ('scan', scan_topic),  # 2D LiDAR - use LaunchConfiguration
        ],
        arguments=['-d'],  # Delete database on start (mapping mode)
        output='screen'
    )

    # RViz2
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
        use_sim_time_arg,
        rviz_arg,
        scan_topic_arg,
        rgbd_odometry,
        rtabmap_slam,
        rviz_node,
    ])
