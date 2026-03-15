# Issues & Fixes

## 2026-03-13

### [FIX] launch 파일 하드코딩 경로 수정

**증상:** `ros2 launch tm_gazebo gazebo.launch.py` 실행 시 world/model/rviz 파일을 찾지 못함

**원인:** launch 파일 내 `pkg_src` 경로가 이전 워크스페이스로 하드코딩됨

```python
# 수정 전
pkg_src = os.path.join(os.path.expanduser('~'), 'T-Robot_nav_ros2_ws', 'src', 'Gazebo')

# 수정 후
pkg_src = os.path.join(os.path.expanduser('~'), 'Study', 'ros2_3dslam_ws', 'src', 'Gazebo')
```

**수정 파일:**

- `src/Gazebo/launch/gazebo.launch.py`
- `src/Gazebo/launch/gazebo_no_odom.launch.py`

---

### [FIX] SLAM 전체 소스 경로 일괄 수정

**원인:** `T-Robot_nav_ros2_ws` → `Study/ros2_3dslam_ws` 워크스페이스 이전 후 경로 미반영

**수정 파일 (31개):**
- `src/SLAM/3D_SLAM/rtab_map_3d/launch/*.launch.py` (15개)
- `src/SLAM/2D_SLAM/rtab_map/launch/*.launch.py` (7개)
- `src/SLAM/2D_SLAM/SLAMBOX/launch/*.launch.py` (3개)
- `src/SLAM/2D_SLAM/Cartographer/launch/*.launch.py` (1개)
- `src/SLAM/2D_SLAM/k_slam_ros2/k_mapping/launch/k_slam.launch.py`
- `src/SLAM/2D_SLAM/k_slam_ros2/k_mapping/src/K_MappingRos.cpp`
- `src/SLAM/SLAM_Manager/2D/slam_manager_2d/slam_manager_2d_node.py`

특수 패턴도 처리:
```python
# livox 파일 (수정 전)
'T-Robotics', 'T-Robot_nav_ros2_ws', 'src', ...
# 수정 후
'Study', 'ros2_3dslam_ws', 'src', ...
```

---

### [FIX] RViz2 3D LiDAR 포인트 클라우드 미표시

**증상:** Gazebo 실행 시 RViz2에 3D 포인트 클라우드가 표시되지 않음

**원인:** `LidarPointCloud` 디스플레이가 `Enabled: false`로 설정되어 있음

**수정 파일:** `rviz/gazebo.rviz`

```yaml
# 수정 전
Name: LidarPointCloud
Enabled: false

# 수정 후
Name: LidarPointCloud
Enabled: true
```

토픽: `/scan/points` (PointCloud2, Reliable QoS)

---

### [분석] RViz2 3D SLAM cloud 추가 시 크래시

**증상:** RViz2에 3D SLAM 포인트 클라우드 추가 시 RViz2 종료

**원인: QoS 불일치**

| 토픽 | Publisher | RViz2 기본 | 상태 |
|------|-----------|-----------|------|
| `/rtabmap/cloud_map` | Reliable + **Transient Local** | Reliable + **Volatile** | ❌ |
| `/hdl_graph_slam/map_points` | **Best Effort** + Volatile | **Reliable** + Volatile | ❌ |

**해결:** RViz2에서 해당 토픽 추가 시 QoS 수동 설정

- `/rtabmap/cloud_map`: Durability → `Transient Local`
- `/hdl_graph_slam/map_points`: Reliability → `Best Effort`

---

## 2026-03-15

### [FIX] ros2 launch 서브디렉토리 경로 사용 불가

**증상:** `ros2 launch cartographer_slam_3d slam/cartographer_3d_slam_gazebo.launch.py` → "not found"

**원인:** `ros2launch` API (`/opt/ros/humble/lib/python3.10/site-packages/ros2launch/api/api.py`)에서 `os.walk`로 파일 검색 시 `name`은 **베이스 파일명만** 포함. `slam/file.py`와 비교하면 절대 매칭 안 됨.

```python
# ros2launch 소스코드
for root, dirs, files in os.walk(package_share_directory):
    for name in files:       # name = "file.py" (경로 없음)
        if name == file_name: # file_name = "slam/file.py" → 매칭 실패
```

**해결:** 서브디렉토리 없이 베이스 파일명만 사용:
```bash
# 수정 전 (동작 안 함)
ros2 launch cartographer_slam_3d slam/cartographer_3d_slam_gazebo.launch.py

# 수정 후 (정상 동작)
ros2 launch cartographer_slam_3d cartographer_3d_slam_gazebo.launch.py
```

**CMakeLists.txt도 수정** (`install(DIRECTORY launch/ ...)` 트레일링 슬래시):
```cmake
# 수정 전: launch 디렉토리 자체가 설치됨 → share/pkg/launch/slam/file.py
install(DIRECTORY launch DESTINATION ...)

# 수정 후: launch 내용물만 설치 → share/pkg/slam/file.py
install(DIRECTORY launch/ DESTINATION ...)
```

**수정 파일:**
- `src/SLAM/3D_SLAM/cartographer_slam_3d/CMakeLists.txt`
- `src/SLAM/2D_SLAM/Cartographer/CMakeLists.txt`
- `src/SLAM/3D_SLAM/rtab_map_3d/CMakeLists.txt`
- `src/SLAM/2D_SLAM/rtab_map/CMakeLists.txt`
- 전체 `docs/*_run_commands.md` (서브디렉토리 경로 제거)

---

### [FIX] colcon --symlink-install 스테일 파일 중복

**증상:** CMakeLists.txt 변경 후 빌드 시 `MultipleLaunchFilesError: file found more than once`

**원인:** `colcon build --symlink-install`는 이전 설치 파일을 정리하지 않음. CMakeLists 변경으로 설치 경로가 바뀌면 **이전 파일 + 새 파일** 모두 존재.

```
# 이전: share/pkg/launch/slam/file.py  (남아 있음)
# 새로: share/pkg/slam/file.py         (새로 설치됨)
# os.walk가 2개 찾음 → MultipleLaunchFilesError
```

**해결:** 클린 빌드:
```bash
rm -rf install/<pkg> build/<pkg>
colcon build --symlink-install --packages-select <pkg>
```

---

### [FIX] Cartographer 3D SLAM - cartographer_node 즉시 크래시

**증상:** `ros2 launch cartographer_slam_3d cartographer_3d_slam_gazebo.launch.py` 실행 시 cartographer_node 즉시 종료 (exit code -6)

**원인:** `ros_arguments`에 `--qos-override` 사용 → ROS2 Humble의 cartographer_node가 인식 못 함

```
rclcpp::exceptions::UnknownROSArgsError:
  found unknown ROS arguments: '--qos-override', '/imu/data:reliability:best_effort'
```

**해결:** `ros_arguments` 제거. IMU QoS 문제는 relay 스크립트로 해결:
- `scripts/imu_qos_relay.py`: BEST_EFFORT로 `/imu/data` 수신 → RELIABLE로 `/imu/data_reliable` 재발행
- cartographer remap: `('imu', '/imu/data_reliable')`

**수정 파일:**
- `src/SLAM/3D_SLAM/cartographer_slam_3d/launch/slam/cartographer_3d_slam_gazebo.launch.py`
- `src/SLAM/3D_SLAM/cartographer_slam_3d/launch/localization/cartographer_3d_localization_gazebo.launch.py`
- `src/SLAM/3D_SLAM/cartographer_slam_3d/scripts/imu_qos_relay.py` (신규)

---

### [FIX] Cartographer 3D TF 체인 끊김 (lidar_link frame dropping)

**증상:** RViz2에서 `Message Filter dropping message: frame 'lidar_link' ... queue is full` 반복

**원인:** lua 설정에서 `provide_odom_frame = false` + `published_frame = "odom"` → Cartographer가 `map → odom` TF만 발행하고 `odom → base_link`는 발행 안 함 → TF 체인 끊김

```
필요한 TF 체인: map → odom → base_link → lidar_link
                          ↑ 여기가 없음
```

**해결:**
```lua
-- 수정 전
published_frame = "odom",
provide_odom_frame = false,

-- 수정 후
published_frame = "base_link",
provide_odom_frame = true,
```

**수정 파일:**
- `src/SLAM/3D_SLAM/cartographer_slam_3d/config/cartographer_3d.lua`
- `src/SLAM/3D_SLAM/cartographer_slam_3d/config/cartographer_3d_localization.lua`

---

### [FIX] Gazebo IMU 센서 데이터 미발행

**증상:** Pioneer2dx SDF에 IMU 센서 추가했으나 `/imu/data` 토픽에 데이터 없음. `ign topic -l`에도 IMU 토픽 없음.

**원인:** World SDF (`my_world.sdf`)에 `ignition-gazebo-imu-system` 플러그인 누락. Ignition Gazebo에서 IMU 센서는 모델 SDF에 `<sensor type="imu">`만으로는 부족하고, **World에 IMU 시스템 플러그인**이 필수.

**해결:**
```xml
<!-- src/Gazebo/worlds/my_world.sdf에 추가 -->
<plugin
  filename="ignition-gazebo-imu-system"
  name="ignition::gazebo::systems::Imu">
</plugin>
```

**수정 파일:**
- `src/Gazebo/worlds/my_world.sdf`

**참고:** World 파일은 Gazebo 시작 시에만 로드되므로 Gazebo 재시작 필요.

---

### [FIX] odom TF 충돌로 3D SLAM 맵 회전/드리프트

**증상:** Cartographer 3D / LIO-SAM 실행 시 시간이 지나면 맵이 회전하고 드리프트 발생

**원인:** Gazebo의 `odom_to_tf` 스크립트와 SLAM 노드가 **동시에 `odom → base_link` TF를 발행**. 두 소스가 경쟁하면서 TF가 교대로 바뀌어 맵 드리프트 발생.

```
충돌 구조:
  odom_to_tf:     odom → base_link (휠 오도메트리 기반)
  Cartographer:   odom → base_link (IMU + LiDAR 기반)
  → 두 TF가 교대되며 맵 회전
```

**해결:** `odom_tf:=false` 플래그로 Gazebo의 odom TF 발행 비활성화:

```bash
# 3D SLAM 사용 시 (자체 odom 제공하는 SLAM)
ros2 launch tm_gazebo gazebo_no_odom.launch.py odom_tf:=false
```

**odom_tf 설정 규칙:**

| SLAM 종류 | odom 소스 | Gazebo 실행 | odom_tf |
|-----------|----------|------------|---------|
| **Cartographer 3D** | 자체 (IMU+LiDAR) | `gazebo_no_odom.launch.py` | `false` (필수) |
| **LIO-SAM** | 자체 (IMU+LiDAR) | `gazebo_no_odom.launch.py` | `false` (필수) |
| **RTAB-Map 3D** | 자체 ICP (odom_rtabmap 프레임) | `gazebo_no_odom.launch.py` | `true` (충돌 없음) |
| **Cartographer 2D** | 외부 필요 (/odom) | `gazebo.launch.py` | `true` (기본값) |
| **RTAB-Map 2D** | 외부 필요 (/odom) | `gazebo.launch.py` | `true` (기본값) |
| **SLAM Toolbox** | 외부 필요 (/odom) | `gazebo.launch.py` | `true` (기본값) |
| **Hector SLAM** | 불필요 | `gazebo.launch.py` | `true` (기본값) |

**핵심 원칙:** SLAM이 자체 odom을 `odom` 프레임으로 발행하면, Gazebo의 odom TF를 반드시 끄세요.
