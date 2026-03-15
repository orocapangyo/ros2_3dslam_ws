# SLAM Toolbox / Hector Mapping 사용법

패키지명: `slambox_config`

## 알고리즘 비교

| 항목 | SLAM Toolbox | Hector Mapping |
|------|-------------|----------------|
| Odometry 필요 | 선택 (권장) | **불필요** |
| 루프 클로저 | 지원 | 미지원 |
| Localization 모드 | 지원 | 미지원 |
| 지도 직렬화 | `.posegraph` + `.pgm` | `.pgm` |
| 적합한 환경 | 일반적인 실내 | 좁고 구조적인 환경 |

**SLAM Toolbox** — ROS2 공식 권장 2D SLAM. Karto SLAM 기반, 루프 클로저와 맵 재사용을 지원합니다.

**Hector Mapping** — Odometry 없이 LiDAR 스캔만으로 SLAM. 빠른 스캔 매칭 기반으로 드론·불안정한 플랫폼에서도 동작합니다.

## 빌드

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select slambox_config
source install/setup.bash
```

## 실행 순서

### 1단계: Gazebo 시뮬레이터 기동

> SLAM Toolbox / Hector는 외부 odom이 필요합니다(Hector는 선택).
> **`odom_tf:=false`를 사용하지 마세요** — 2D SLAM은 Gazebo의 odom TF가 필수입니다.

```bash
ros2 launch tm_gazebo gazebo.launch.py
```

### 2단계: SLAM 기동

#### SLAM (매핑)

```bash
# SLAM Toolbox
ros2 launch slambox_config slam_toolbox_slam.launch.py

# Hector Mapping
ros2 launch slambox_config hector_slam.launch.py
```

#### Localization (위치추정 - SLAM Toolbox 전용)

```bash
ros2 launch slambox_config slam_toolbox_localization.launch.py
```

## 실행 옵션

| 인수 | 기본값 | 설명 |
|------|--------|------|
| `use_sim_time` | `true` | 시뮬레이션 시간 사용 |
| `scan_topic` | `/scan` | 레이저 스캔 토픽 |
| `rviz` | `true` | RViz 실행 여부 |

## 지도 저장 (SLAM Toolbox)

```bash
ros2 run nav2_map_server map_saver_cli -f ~/Study/ros2_3dslam_ws/maps/slam_toolbox_map
```

## 필수 토픽

| 토픽 | 타입 |
|------|------|
| `/scan` | `sensor_msgs/LaserScan` |
| `/odom` | `nav_msgs/Odometry` (SLAM Toolbox 권장) |
