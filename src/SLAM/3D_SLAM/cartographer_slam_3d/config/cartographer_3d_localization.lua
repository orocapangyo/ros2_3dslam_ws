-- Cartographer 3D Localization configuration
-- Gazebo Pioneer2dx simulation
-- 기존 .pbstream 지도 파일 기반 위치 추정 모드

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
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

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.min_range = 0.3
TRAJECTORY_BUILDER_3D.max_range = 50.0
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10.0
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false

TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter = {
  max_length = 2.0,
  min_num_points = 150,
  max_range = 15.0,
}

TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter = {
  max_length = 4.0,
  min_num_points = 200,
  max_range = 50.0,
}

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

-- Pure localization: 기존 지도에서 위치추정만 수행 (새 서브맵 추가 안함)
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

POSE_GRAPH.optimize_every_n_nodes = 3
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60

return options
