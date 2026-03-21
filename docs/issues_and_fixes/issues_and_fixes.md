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

### [FIX] Cartographer 3D TF 부모 프레임 충돌 → 맵 기울어짐/회전

**증상:** Cartographer 3D 실행 시 맵이 20-30도 기울어지고, 이동하면 맵이 계속 회전. RViz2에서 `Message Filter dropping message: frame 'lidar_link' ... queue is full`

**진단 과정:**

1. 초기 설정: `published_frame = "odom"`, `provide_odom_frame = false` → `odom → base_link` TF 없음
2. 1차 수정: `published_frame = "base_link"`, `provide_odom_frame = true` → **여전히 맵 회전**
3. `ros2 topic info -v /imu/data` → QoS: RELIABLE (QoS 불일치 아님)
4. IMU 데이터 정상: `linear_acceleration.z = 9.80` (중력), rate 199.92 Hz
5. **`ros2 run tf2_tools view_frames`로 TF 트리 확인 → `map`, `odom` 프레임 아예 없음!**

**근본 원인:** TF2 부모 프레임 충돌

```
Static TF:      base_footprint → base_link  (항상 발행됨)
Cartographer:   odom → base_link            (published_frame = "base_link")
                ↑ 같은 child(base_link)에 부모가 2개 → TF2가 Cartographer TF 무시
                → map, odom 프레임이 TF 트리에 등장하지 않음
                → RViz가 map 프레임 기준으로 포인트클라우드 변환 불가 → 맵 회전
```

**해결:** `published_frame`을 `"base_footprint"`로 변경 → 충돌 없는 TF 체인 구성

```lua
-- 수정 전 (TF 충돌)
published_frame = "base_link",
provide_odom_frame = true,

-- 수정 후 (정상)
published_frame = "base_footprint",
provide_odom_frame = true,
```

```
정상 TF 체인: map → odom → base_footprint → base_link → lidar_link
              (Cartographer)    (Cartographer)    (Static TF)    (Static TF)
              각 프레임의 부모가 하나씩 → 충돌 없음
```

**검증 결과:**
- `tf2_echo map base_link`: RPY = [0.24°, 0.16°, -1.85°] (수평 유지)
- `view_frames`: `map → odom → base_footprint → base_link → lidar_link` (정상 트리)
- 맵 누적: 72,349 포인트 / 699 스캔 정상 누적

**수정 파일:**
- `src/SLAM/3D_SLAM/cartographer_slam_3d/config/cartographer_3d.lua`
- `src/SLAM/3D_SLAM/cartographer_slam_3d/config/cartographer_3d_localization.lua`

**핵심 교훈:** `published_frame`은 반드시 **static TF가 점유하지 않는 프레임**이어야 함. `base_link`처럼 이미 다른 부모(`base_footprint`)가 있는 프레임을 지정하면 Cartographer의 TF 발행이 무시됨.

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

---

## 2026-03-22

### [REFACTOR] SLAM 패키지 rviz → rviz2 폴더 표준화

**배경:** 각 SLAM 패키지의 RViz 설정 폴더 이름이 `rviz/`로 되어 있어 ROS2 도구명(`rviz2`)과 불일치. LIO-SAM은 `config/rviz2.rviz`에 별도 위치.

**변경 내용:**

1. **폴더 이름 변경** (7개 패키지):

| 패키지 | 변경 전 | 변경 후 |
|--------|---------|---------|
| Cartographer 2D | `rviz/` | `rviz2/` |
| SLAMBOX | `rviz/` | `rviz2/` |
| Hector SLAM | `rviz/` | `rviz2/` |
| RTAB-Map 2D | `rviz/` | `rviz2/` |
| RTAB-Map 3D | `rviz/` | `rviz2/` |
| LIO-SAM | `config/rviz2.rviz` | `rviz2/lio_sam.rviz` |
| Cartographer 3D | `rviz/` | `rviz2/` |

2. **Launch 파일 일괄 수정** (53개):
   - 모든 `os.path.join(pkg_src, 'rviz', ...)` → `os.path.join(pkg_src, 'rviz2', ...)`
   - LIO-SAM: `os.path.join(src_dir, 'config', 'rviz2.rviz')` → `os.path.join(src_dir, 'rviz2', 'lio_sam.rviz')`

3. **파일명 수정**: `hector_slam_ros2/rviz/k_slam.rviz` → `hector_slam_ros2/rviz2/hector_slam.rviz` (launch 파일과 일치하도록)

---

### [FIX] RViz config 필수 토픽 누락 보완

**증상:** 일부 rviz config에 Odometry 디스플레이 없어 odom 시각화 불가

**검토 결과** (16개 config 전수 검토):

| Config | Map | Sensor | Odom | TF | 조치 |
|--------|-----|--------|------|----|------|
| Cartographer 2D | `/map` | `/scan` | **누락** | O | `/odom` 추가 |
| Hector SLAM (hector_slam_ros2) | `/map` | `/scan` | **누락** | O | `/odom` 추가 |
| SLAMBOX hector_slam | `/map` | `/scan` | **누락** | O | `/odom` 추가 |
| LIO-SAM | `/lio_sam/mapping/map_global` | `/lio_sam/mapping/cloud_registered` | **누락** | O | `/lio_sam/mapping/odometry` 추가 |
| SLAMBOX slam_toolbox | `/map` | `/scan` | `/odom` | O | OK |
| RTAB-Map 2D (2개) | `/rtabmap/map` | `/scan_merged` | `/rtabmap/odom` | O | OK |
| RTAB-Map 3D (8개) | `/rtabmap/map` | 각 센서별 | `/rtabmap/odom` | O | OK |
| Cartographer 3D | `/map_cloud` | `/scan/points` | `/tracked_pose` | O | OK |

**수정 파일:**
- `src/SLAM/2D_SLAM/Cartographer/rviz2/cartographer.rviz`
- `src/SLAM/2D_SLAM/hector_slam_ros2/rviz2/hector_slam.rviz`
- `src/SLAM/2D_SLAM/SLAMBOX/rviz2/hector_slam.rviz`
- `src/SLAM/3D_SLAM/LIO-SAM/livox_lio_sam/rviz2/lio_sam.rviz`

**필수 디스플레이 체크리스트:** 모든 SLAM rviz config에는 최소 Grid, TF, Map(또는 PointCloud2 맵), Sensor(LaserScan/PointCloud2), Odometry가 포함되어야 함.

---

### [FIX] LIO-SAM odom→lidar_link TF 중복 경로

**증상:** LIO-SAM + Gazebo 실행 시 TF 트리에 `odom`에서 `lidar_link`까지 두 경로 존재

**원인:** `mapOptmization.cpp`에서 `odom → lidar_link` TF를 항상 발행. Gazebo 정적 TF `base_link → lidar_link`와 결합하면 중복:
1. `odom → base_link → lidar_link` (TransformFusion + Gazebo)
2. `odom → lidar_link` (mapOptimization 직접 발행)

**해결:** `mapOptmization.cpp:1901`에서 조건부 발행. `lidarFrame == baselinkFrame`일 때만:

```cpp
if (lidarFrame == baselinkFrame) {
    br->sendTransform(trans_odom_to_lidar);
}
```

**수정 파일:** `src/SLAM/3D_SLAM/LIO-SAM/livox_lio_sam/src/mapOptmization.cpp`

---

### [FIX] LIO-SAM RViz config 경로 오류 (dirname 깊이 부족)

**증상:** RViz2에 Grid만 표시. 타이틀바: `.../launch/config/rviz2.rviz` (잘못된 경로)

**원인:** 론치파일이 `launch/slam/` (2단계 깊이)에 위치. `os.path.dirname` 2번은 `launch/`까지만 올라감.

```python
# 수정 전: launch/ 까지만 올라감
src_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

# 수정 후: 패키지 루트까지 올라감
src_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
```

**수정 파일:** 4개 론치파일 전부

---

### [FIX] YAML config 절대경로 → 이식 가능 경로

**증상:** `savePCDDirectory`, `globalMapPath`에 `/home/amap/...` 절대경로 → 다른 PC 동작 불가

**해결:** YAML에 `~/` 경로, 론치파일에서 `os.path.expanduser()`로 확장 후 파라미터 오버라이드:

```python
save_pcd_dir = os.path.expanduser('~/Study/ros2_3dslam_ws/maps/lio_sam/')
Node(..., parameters=[parameter_file, {'savePCDDirectory': save_pcd_dir}])
```

**수정 파일:**
- `config/params_slam_gazebo.yaml`, `config/params_localization_gazebo.yaml`
- `launch/slam/run_slam_gazebo.launch.py`
