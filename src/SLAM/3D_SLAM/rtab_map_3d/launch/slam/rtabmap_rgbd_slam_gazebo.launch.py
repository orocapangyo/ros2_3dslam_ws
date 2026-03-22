"""
RTAB-Map 3D RGB-D Mapping Launch File

This launch file is for RGB-D SLAM using Orbbec Astra Pro camera.
Uses visual odometry (RGBD Odometry) instead of ICP odometry.

User Configuration:
- Orbbec camera only (no LiDAR)
- base_link = camera_link (same position)

Required Topics:
- /camera/color/image_raw (sensor_msgs/Image)
- /camera/color/camera_info (sensor_msgs/CameraInfo)
- /camera/depth/image_raw (sensor_msgs/Image)
- /camera/depth/camera_info (sensor_msgs/CameraInfo)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for RGB-D SLAM."""

    # src folder path (CLAUDE.md rule)
    pkg_src = os.path.join(
        os.path.expanduser('~'),
        'Study', 'ros2_3dslam_ws', 'src', 'SLAM', '3D_SLAM', 'rtab_map_3d'
    )
    rviz_config = os.path.join(pkg_src, 'rviz2', 'rtabmap_rgbd.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
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

    # Gazebo provides odom->base_link and base_link->camera_link TFs
    # No additional static transforms needed for Gazebo simulation
    # (static_transforms.launch.py is for real Orbbec camera only)

    # RGBD Odometry node (Visual Odometry for RGB-D camera)
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'wait_for_transform': 0.2,
            'approx_sync': True,
            'queue_size': 10,
            # QoS: BEST_EFFORT to match Gazebo bridge output
            'qos_image': 2,
            'qos_camera_info': 2,
            # Visual odometry parameters
            'Odom/Strategy': '0',  # 0=Frame-to-Map, 1=Frame-to-Frame
            'Odom/ResetCountdown': '1',
            'Vis/FeatureType': '6',  # 6=ORB
            'Vis/CorType': '0',  # 0=Features Matching
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

    # RTABMAP SLAM node (RGB-D mode)
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
            # Sensor subscription settings (RGB-D mode)
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_rgbd': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': False,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'queue_size': 10,
            # QoS: BEST_EFFORT to match Gazebo bridge output
            'qos_image': 2,
            'qos_camera_info': 2,
            # RTAB-Map parameters (RGB-D SLAM)
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            # Loop closure detection
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            # Registration (ICP + Visual)
            'Reg/Strategy': '0',  # 0=Visual, 1=ICP, 2=Visual+ICP
            'Reg/Force3DoF': 'false',
            # Grid/Map parameters
            'Grid/RangeMax': '5.0',  # RGB-D range limit
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
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
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
        rgbd_odometry,
        rtabmap_slam,
        rviz_node,
    ])
