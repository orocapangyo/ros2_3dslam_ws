-- Cartographer 3D SLAM configuration
-- Gazebo Pioneer2dx simulation
-- Sensors: 16-channel 3D LiDAR (/scan/points) + IMU (/imu/data)

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",       -- IMU frame (base_link에 IMU 장착)
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,  -- 3D이므로 false
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,                    -- 2D LaserScan 없음
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,                   -- 3D PointCloud2 사용
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- 3D SLAM 모드 활성화
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4

-- 3D Trajectory Builder 파라미터 (Pioneer2dx 16ch LiDAR 튜닝)
TRAJECTORY_BUILDER_3D.min_range = 0.3
TRAJECTORY_BUILDER_3D.max_range = 50.0
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15

-- IMU 설정 (Gazebo 시뮬레이션 IMU)
-- imu_gravity_time_constant: 낮을수록 중력 방향 빠르게 수렴
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 0.5
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

-- 고해상도 적응형 복셀 필터
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter = {
  max_length = 2.0,
  min_num_points = 150,
  max_range = 15.0,
}

-- 저해상도 적응형 복셀 필터
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter = {
  max_length = 4.0,
  min_num_points = 200,
  max_range = 50.0,
}

-- Ceres scan matcher
TRAJECTORY_BUILDER_3D.ceres_scan_matcher = {
  occupied_space_weight_0 = 1.0,
  occupied_space_weight_1 = 6.0,
  translation_weight = 5.0,
  rotation_weight = 4e2,
  only_optimize_yaw = false,
  ceres_solver_options = {
    use_nonmonotonic_steps = false,
    max_num_iterations = 12,
    num_threads = 1,
  },
}

-- Pose Graph 최적화
POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5

return options
