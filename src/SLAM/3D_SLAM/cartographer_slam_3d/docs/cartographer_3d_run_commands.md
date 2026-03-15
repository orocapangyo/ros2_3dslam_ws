# Cartographer 3D SLAM 실행 가이드

Gazebo Pioneer2dx 시뮬레이션 기반 Cartographer 3D SLAM/Localization 사용법

## 센서 구성 (Gazebo Pioneer2dx)

| 센서 | 토픽 | 타입 | 주파수 |
|------|------|------|--------|
| 3D LiDAR (16ch) | `/scan/points` | PointCloud2 | 20 Hz |
| IMU | `/imu/data` | Imu | 200 Hz |

> Cartographer 3D는 IMU 필수. Pioneer2dx SDF에 IMU 추가됨 (200Hz, base_link 프레임)

---

## 패키지 구조

```
cartographer_slam_3d/
├── config/
│   ├── cartographer_3d.lua              # 3D SLAM 설정
│   └── cartographer_3d_localization.lua # 3D Localization 설정
├── launch/
│   ├── slam/
│   │   └── cartographer_3d_slam_gazebo.launch.py
│   └── localization/
│       └── cartographer_3d_localization_gazebo.launch.py
└── docs/
    └── cartographer_3d_run_commands.md
```

---

## Gazebo 사전 준비

> **odom_tf:=false 필수!** Cartographer 3D는 자체적으로 `odom → base_link` TF를 발행합니다.
> `odom_tf:=false`를 빼면 Gazebo의 odom_to_tf와 충돌하여 **맵이 회전/드리프트**합니다.

```bash
# 터미널 1: Gazebo 시뮬레이션 실행
ros2 launch tm_gazebo gazebo_no_odom.launch.py odom_tf:=false
```

---

## 3D SLAM 모드 (지도 생성)

```bash
# 터미널 2: Cartographer 3D SLAM 실행
ros2 launch cartographer_slam_3d cartographer_3d_slam_gazebo.launch.py

# 터미널 3: 로봇 조작 (rqt 또는 키보드)
rqt   # Robot Steering 플러그인 사용
```

### 지도 저장 (.pbstream)

```bash
# 매핑 완료 후 지도 저장
mkdir -p ~/Study/ros2_3dslam_ws/maps/cartographer_3d/

ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$(echo $HOME)/Study/ros2_3dslam_ws/maps/cartographer_3d/map.pbstream', \
    include_unfinished_submaps: true}"
```

---

## 3D Localization 모드 (위치 추정)

```bash
# 저장된 지도로 localization 실행
ros2 launch cartographer_slam_3d cartographer_3d_localization_gazebo.launch.py

# 커스텀 지도 파일 지정
ros2 launch cartographer_slam_3d cartographer_3d_localization_gazebo.launch.py \
  map_file:=/path/to/map.pbstream
```

---

## 설정 파일 비교

| 항목 | cartographer_3d.lua | cartographer_3d_localization.lua |
|------|--------------------|---------------------------------|
| 모드 | 3D SLAM (지도 생성) | 3D Localization (위치 추정) |
| `num_point_clouds` | 1 | 1 |
| `use_trajectory_builder_3d` | true | true |
| `pure_localization_trimmer` | 없음 | max_submaps_to_keep: 3 |
| `optimize_every_n_nodes` | 20 | 3 (빠른 수렴) |

---

## 2D Cartographer와 차이점

| 항목 | 2D (cartographer_slam) | 3D (cartographer_slam_3d) |
|------|----------------------|--------------------------|
| 입력 센서 | LaserScan (`/scan`) | PointCloud2 (`/scan/points`) |
| IMU | 선택 (비활성) | 필수 (`/imu/data`) |
| 출력 맵 | 2D Occupancy Grid | 3D Submaps |
| 위치 | `src/SLAM/2D_SLAM/` | `src/SLAM/3D_SLAM/` |
| lua 설정 | `TRAJECTORY_BUILDER_2D` | `TRAJECTORY_BUILDER_3D` |
| `publish_frame_projected_to_2d` | true | false |

---

## 지도 파일 경로

```
~/Study/ros2_3dslam_ws/maps/cartographer_3d/map.pbstream
```

---

## 빌드

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --symlink-install --packages-select cartographer_slam_3d
source install/setup.bash
```
