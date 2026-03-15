# RTAB-Map 2D SLAM 사용법

패키지명: `rtab_map_config`

## 개요

RTAB-Map(Real-Time Appearance-Based Mapping)은 그래프 기반 SLAM으로 메모리 관리를 통해 대규모 환경에서도 실시간 동작합니다.
ICP Odometry로 LiDAR 스캔 매칭 기반 주행거리를 계산하며, RGB-D 카메라를 추가하면 시각적 루프 클로저로 정확도가 향상됩니다.

## 모드 선택

| 모드 | launch 위치 | 센서 | 특징 |
|------|------------|------|------|
| LiDAR 전용 | `slam/` | `/scan` | 단순·빠름 |
| LiDAR + Camera | `slam/` | `/scan` + `/camera/color/image_raw` | 시각적 루프 클로저 |
| LiDAR + RGB-D | `slam/` | `/scan` + `/camera/depth/image_raw` | 3D 장애물 인식 + 루프 클로저 |

## 빌드

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select rtab_map_config
source install/setup.bash
```

## 실행 순서

### 1단계: Gazebo 시뮬레이터 기동

> RTAB-Map 2D는 외부 odom이 필요합니다. `gazebo.launch.py`는 `/odom` 토픽과 `odom → base_link` TF를 제공합니다.
> **`odom_tf:=false`를 사용하지 마세요** — 2D SLAM은 Gazebo의 odom TF가 필수입니다.

```bash
ros2 launch tm_gazebo gazebo.launch.py
```

### 2단계: SLAM 기동

#### SLAM (매핑)

```bash
# LiDAR 전용
ros2 launch rtab_map_config rtabmap_lidar_slam.launch.py

# LiDAR + Camera
ros2 launch rtab_map_config rtabmap_lidar_camera_slam.launch.py

# LiDAR + RGB-D
ros2 launch rtab_map_config rtabmap_2d_lidar_rgbd_slam.launch.py
```

#### Localization (위치추정 - 기존 지도 재사용)

```bash
# LiDAR 전용
ros2 launch rtab_map_config rtabmap_lidar_localization.launch.py

# LiDAR + Camera
ros2 launch rtab_map_config rtabmap_lidar_camera_localization.launch.py

# LiDAR + RGB-D
ros2 launch rtab_map_config rtabmap_2d_lidar_rgbd_localization.launch.py
```

## 실행 옵션

| 인수 | 기본값 | 설명 |
|------|--------|------|
| `use_sim_time` | `true` | 시뮬레이션 시간 사용 |
| `scan_topic` | `/scan` | 레이저 스캔 토픽 |
| `database_path` | `maps/rtabmap_2d/rtabmap.db` | DB 파일 경로 (localization 전용) |
| `rviz` | `true` | RViz 실행 여부 |

## 지도 파일

```bash
~/Study/ros2_3dslam_ws/maps/rtabmap_2d/rtabmap.db
```

매핑 완료 후 자동 저장. 위치추정 시 동일 경로 사용.

## 필수 토픽

| 토픽 | 타입 | 모드 |
|------|------|------|
| `/scan` | `sensor_msgs/LaserScan` | 전체 |
| `/odom` | `nav_msgs/Odometry` | 전체 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | Camera/RGB-D 모드 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | RGB-D 모드 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Camera/RGB-D 모드 |
