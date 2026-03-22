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

---

### [NEW] RTAB-Map 3D: 3개 센서 조합 Gazebo launch 파일 추가

**내용:** 기존 Livox 전용(`/livox/lidar`) 파일 외에 Gazebo 범용 3D LiDAR(`/scan/points`) 기반 launch 파일 6개 추가.

**센서 조합:**

| 조합 | SLAM | Localization |
|------|------|--------------|
| 3D LiDAR + RGBD | `rtabmap_3dlidar_rgbd_slam_gazebo.launch.py` | `rtabmap_3dlidar_rgbd_localization_gazebo.launch.py` |
| 3D LiDAR only | `rtabmap_3dlidar_only_slam_gazebo.launch.py` | `rtabmap_3dlidar_only_localization_gazebo.launch.py` |
| 3D LiDAR + RGB | `rtabmap_3dlidar_rgb_slam_gazebo.launch.py` | `rtabmap_3dlidar_rgb_localization_gazebo.launch.py` |

**핵심 설계:**
- `frame_id: base_footprint`, `odom_frame_id: odom_rtabmap`
- ICP odom과 SLAM 노드 모두 `/scan/points` 사용 (기존 livox 파일의 토픽 불일치 버그 방지)
- RGBD 변형: `point_cloud_xyzrgb` + `point_cloud_assembler` + `TimerAction` RViz
- `Reg/Strategy`: RGBD/RGB = 2 (ICP+Visual), LiDAR-only = 1 (ICP only)
- Localization: `database_path` 인수, `Mem/IncrementalMemory: false`, `Optimizer/Robust: true`

**추가 수정:**
- `package.xml`: `<exec_depend>rtabmap_util</exec_depend>` 추가
- `docs/rtab_map_3d_run_commands.md`: 6개 새 명령어 추가

---

### [FIX] RTAB-Map 3D CMakeLists.txt: rviz2 디렉토리 참조 누락

**증상:** `colcon build --packages-select rtab_map_3d_config` 실패

```
CMake Error: ament_cmake_symlink_install_directory() can't find
  '.../rtab_map_3d/rviz'
```

**원인:** `rviz/` → `rviz2/` 폴더명 변경 후 CMakeLists.txt 미수정

```cmake
# 수정 전
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

# 수정 후
install(DIRECTORY rviz2 DESTINATION share/${PROJECT_NAME})
```

**수정 파일:** `src/SLAM/3D_SLAM/rtab_map_3d/CMakeLists.txt`

---

### [NOTE] RTAB-Map Localization: loadWordsQuery 시각 단어 사전 경고

**증상:** Localization 모드에서 DB 로드 시 에러 로그 발생

```
[ERROR] DBDriverSqlite3.cpp:3858::loadWordsQuery() Query (5459) doesn't match loaded words (0)
[ERROR] VWDictionary.cpp:741::addWordRef() Not found word 1 (dict size=0)
```

**원인:** 짧은 SLAM 세션(~10초 주행)에서 시각 단어 사전(BoW vocabulary)이 충분히 형성되지 않은 상태로 DB 저장. Localization 시 사전 로드 실패.

**심각도:** 비치명적. ICP 기반 localization은 정상 동작. 시각적 루프 클로저만 비활성화.

**해결:** 더 긴 SLAM 매핑 세션 (1분+ 주행)에서 DB 저장 시 해소됨.

---

### [TEST] RTAB-Map 3D: 6개 launch 파일 Gazebo 통합 테스트 결과

**테스트 환경:** Gazebo Ignition Fortress + Pioneer2dx + `gazebo_no_odom.launch.py odom_tf:=false`

| 테스트 | 결과 | 비고 |
|--------|------|------|
| TF 트리 검증 | **PASS** | `base_footprint→base_link` z=0.16 |
| 센서 토픽/QoS | **PASS** | 7/7 토픽, RELIABLE QoS, depth frame_id 호환 |
| LiDAR only SLAM | **PASS** | DB 9.3MB |
| LiDAR+RGB SLAM | **PASS** | DB 12MB |
| LiDAR+RGBD SLAM | **PASS** | DB 68MB, 4노드, 컬러클라우드 |
| LiDAR only Localization | **PASS** | DB 로드 + TF 정상 |
| LiDAR+RGB Localization | **PASS** | 시각 단어 경고 (비치명적) |
| LiDAR+RGBD Localization | **PASS** | 4노드 + 컬러클라우드 |
| Gazebo 없이 SLAM | **PASS** | 크래시 없음 |
| 잘못된 DB 경로 | **PASS** | 명확한 FATAL 에러 후 종료 |

**Gazebo QoS 발견:** `/scan/points` 등 센서 토픽은 `RELIABLE` QoS로 발행됨 (기존 문서는 `BEST_EFFORT` 가정). RTAB-Map의 `qos_scan_cloud: 2` (BEST_EFFORT) subscriber는 RELIABLE publisher와 호환되므로 문제 없음.

**DB 크기 비교:** LiDAR-only (9.3MB) < LiDAR+RGB (12MB) < LiDAR+RGBD (68MB) — 센서 데이터 양에 비례

---

### [BUG] RTAB-Map 3D: 8개 Gazebo SLAM launch 파일 전수 검사 결과

**테스트 환경:** Gazebo Ignition Fortress + Pioneer2dx + `gazebo_no_odom.launch.py`

**테스트 대상:** `rtab_map_3d_config` 패키지의 Gazebo SLAM launch 파일 8개

| # | Launch File | 결과 | 발견된 이슈 |
|---|------------|------|------------|
| 1 | rtabmap_rgbd_slam_gazebo | **FAIL** | use_sim_time=false, depth_info remap |
| 2 | rtabmap_2dlidar_rgbd_slam_gazebo | **WARN** | depth_info remap |
| 3 | rtabmap_3dlidar_only_slam_gazebo | **PASS** | — |
| 4 | rtabmap_3dlidar_rgb_slam_gazebo | **PASS** | — |
| 5 | rtabmap_3dlidar_rgbd_slam_gazebo | **PASS** | — |
| 6 | rtabmap_livox_rgbd_slam_gazebo | **FAIL** | TF 충돌, 토픽 불일치, QoS 누락 |
| 7 | rtabmap_3d_slam_gazebo | **PASS** | — |
| 8 | rtabmap_astra_3d_slam_gazebo | **FAIL** | TF 충돌 |

---

#### Bug 1: use_sim_time 기본값 오류 (심각도: 높음)

**파일:** `launch/slam/rtabmap_rgbd_slam_gazebo.launch.py:42`

**증상:** 실행 시 `"Odometry: Detected not valid consecutive stamps (previous=27599s new=34s)"` 에러 반복. 타임스탬프 27599초(Gazebo sim time)와 34초(system wall clock)가 혼재.

**원인:** `use_sim_time` 기본값이 `false` — 다른 모든 Gazebo launch 파일은 `true`

```python
# 수정 전 (버그)
default_value='false',

# 수정 필요
default_value='true',
```

**영향:** 모든 노드가 system clock 사용 → TF 시간 불일치 → odometry 데이터 동기화 실패

---

#### Bug 2: TF 부모 프레임 충돌 — base_link (심각도: 높음)

**파일:**
- `launch/slam/rtabmap_livox_rgbd_slam_gazebo.launch.py:74,112`
- `launch/slam/rtabmap_astra_3d_slam_gazebo.launch.py:56,89`

**증상:** Odometry가 `odom → base_link` TF 발행 시도. Gazebo static TF `base_footprint → base_link`과 parent 충돌.

```
충돌 구조 (Cartographer 3D와 동일한 패턴):
  Gazebo Static TF:  base_footprint → base_link  (항상 발행)
  RGBD/ICP Odom:     odom → base_link            (frame_id='base_link')
                     ↑ base_link에 부모가 2개 → TF2가 odom TF 무시
```

**런타임 증거 (Livox):**
```
[icp_odometry] Odometry: frame_id = base_link
[icp_odometry] Odometry: odom_frame_id = odom
[icp_odometry] TF odom->base_link is not published because we detected a time jump
```

**해결 방향:** `frame_id`를 `base_footprint`로 변경 (3dlidar 시리즈와 동일하게)

```python
# 수정 전 (TF 충돌)
'frame_id': 'base_link',
'odom_frame_id': 'odom',

# 수정 후 (정상)
'frame_id': 'base_footprint',
'odom_frame_id': 'odom_rtabmap',  # 또는 'odom' (gazebo_no_odom 사용 시)
```

**기존 사례:** Cartographer 3D에서 `published_frame = "base_link"` → `"base_footprint"` 변경으로 동일 문제 해결 (이 문서 2026-03-15 참조)

---

#### Bug 3: Livox scan_cloud 토픽 불일치 (심각도: 높음)

**파일:** `launch/slam/rtabmap_livox_rgbd_slam_gazebo.launch.py`

**증상:** SLAM 노드가 LiDAR 데이터를 수신하지 못함. `ros2 topic info /livox/lidar` → Publisher 0개, Subscriber 1개.

**원인:** 동일 launch 파일 내 ICP odom과 SLAM 노드가 서로 다른 토픽 사용

```python
# ICP Odometry (L99) — 정상
remappings=[('scan_cloud', '/scan/points')],  # Gazebo가 발행하는 토픽

# RTAB-Map SLAM (L155) — 버그
remappings=[
    ('scan_cloud', '/livox/lidar'),  # Gazebo는 이 토픽 미발행!
]
```

**런타임 증거:**
```
$ ros2 topic info /livox/lidar
Publisher count: 0     ← 데이터 소스 없음
Subscription count: 1  ← SLAM 노드만 대기 중

$ ros2 topic info /scan/points
Publisher count: 2     ← Gazebo bridge 정상 발행
Subscription count: 2  ← ICP odom만 수신
```

**해결:** SLAM 노드 remapping을 `/scan/points`로 변경

```python
# 수정
('scan_cloud', '/scan/points'),
```

---

#### Bug 4: Livox point_cloud_xyzrgb QoS 누락 (심각도: 중간)

**파일:** `launch/slam/rtabmap_livox_rgbd_slam_gazebo.launch.py:174-181`

**원인:** `point_cloud_xyzrgb` 노드에 Gazebo bridge QoS 설정 누락

```python
# 현재 (QoS 미설정 → 기본 RELIABLE)
parameters=[{
    'use_sim_time': use_sim_time,
    'decimation': 4,
    'max_depth': 4.0,
    'voxel_size': 0.02,
    'approx_sync': True,
    'queue_size': 10,
}],

# 수정 필요 (3dlidar_rgbd 파일 참조, L162-164)
'qos_image': 2,        # BEST_EFFORT
'qos_camera_info': 2,  # BEST_EFFORT
```

**영향:** Gazebo bridge 카메라 토픽 수신 불가 → 컬러 포인트 클라우드 미생성

---

#### Note 1: depth/camera_info remapping 불일치 (심각도: 낮음)

**파일:**
- `launch/slam/rtabmap_rgbd_slam_gazebo.launch.py:85`
- `launch/slam/rtabmap_2dlidar_rgbd_slam_gazebo.launch.py:84`

**내용:** `depth/camera_info`를 `/camera/color/camera_info`로 remap (color 카메라 info를 depth info로 사용)

```python
# 현재 (2개 파일)
('depth/camera_info', '/camera/color/camera_info'),

# 3D LiDAR+RGBD 파일들은 올바르게:
('depth/camera_info', '/camera/depth/camera_info'),
```

**영향:** Gazebo에서 color/depth camera_info가 동일한 카메라 모델이면 실질적 차이 없음. 실제 하드웨어(다른 해상도/intrinsics)에서는 문제 발생 가능.

---

#### Note 2: queue_size 파라미터 deprecation (심각도: 낮음)

**증상:** 전체 launch 파일에서 경고 발생

```
Parameter "queue_size" has been renamed to "sync_queue_size" and will be removed in future versions!
```

**해결:** 향후 rtabmap 업데이트 시 파라미터명 일괄 변경 필요 (`queue_size` → `sync_queue_size`)

---

## 2026-03-22

### [TEST] LIO-SAM Localization Mode: Gazebo 통합 테스트 결과

**테스트 환경:** Gazebo Ignition Fortress + Pioneer2dx + `gazebo_no_odom.launch.py`

**테스트 명령:**
```bash
ros2 launch livox_lio_sam run_localization_gazebo.launch.py
```

**사전 조건:** SLAM 모드에서 생성된 맵 파일이 `~/Study/ros2_3dslam_ws/maps/lio_sam/`에 존재

| 항목 | 결과 | 비고 |
|------|------|------|
| 맵 로딩 | **PASS** | Corner: 2684, Surf: 11721, Viz: 11896 points |
| 노드 기동 | **PASS** | 4개 노드 + RViz + static_transform_publisher 정상 기동 |
| Odometry 발행 | **PASS** | `/lio_sam/mapping/odometry` ~3-5 Hz 발행 |
| TF (map→base_link) | **PASS** | 변환 발행 중, 단 불안정 |
| Global Map 시각화 | **PASS** | 기동 직후 `/lio_sam/mapping/map_global` 발행 |
| RViz Odometry 표시 | **FAIL** | QoS 불일치로 표시 불가 |
| Pose 안정성 | **FAIL** | deskew 비활성화로 인한 심한 드리프트 |

---

#### Bug 1: PCD 파일명 불일치 — save_map vs auto-save (심각도: 중간)

**증상:** Localization 모드 시작 시 `CornerMap.pcd`, `SurfMap.pcd` 파일을 찾지 못함 (fallback으로 `GlobalMap.pcd` 시도)

**원인:** `mapOptmization.cpp`에 두 가지 맵 저장 경로가 존재하며, 파일명이 다름

| 저장 방식 | 코드 위치 | 파일명 | 형식 |
|-----------|----------|--------|------|
| `/lio_sam/save_map` 서비스 | L217-255 | `GlobalMap.pcd`, `CornerMap.pcd`, `SurfMap.pcd` | Binary |
| Ctrl+C 종료 시 auto-save | L605-625 | `cloudGlobal.pcd`, `cloudCorner.pcd`, `cloudSurf.pcd` | ASCII |

`loadGlobalMap()` (L343-347)은 서비스 방식의 파일명(`CornerMap.pcd`, `SurfMap.pcd`)을 기대함.

**해결:** SLAM 모드에서 맵 저장 시 반드시 서비스 호출 사용:
```bash
ros2 service call /lio_sam/save_map livox_lio_sam/srv/SaveMap "{resolution: 0.2, destination: ''}"
```

또는 auto-save 파일을 수동 복사:
```bash
cd ~/Study/ros2_3dslam_ws/maps/lio_sam/
cp cloudGlobal.pcd GlobalMap.pcd
cp cloudCorner.pcd CornerMap.pcd
cp cloudSurf.pcd SurfMap.pcd
```

---

#### Bug 2: QoS 불일치 — RViz에서 Odometry/Path 표시 불가 (심각도: 중간)

**증상:** RViz2에서 `/lio_sam/mapping/odometry`, `/lio_sam/imu/path` 토픽 구독 실패

```
[rviz2] New publisher discovered on topic '/lio_sam/mapping/odometry', offering incompatible QoS.
        Last incompatible policy: RELIABILITY_QOS_POLICY
```

**원인:** `utility.hpp:409-419`에서 `qos_profile`의 Reliability가 `BEST_EFFORT`로 설정. RViz2 기본 구독 QoS는 `RELIABLE`.

```
Publisher (LIO-SAM): BEST_EFFORT → RViz2 Subscriber: RELIABLE → 불일치!
```

**영향 토픽:**
- `/lio_sam/mapping/odometry` (mapOptimization, L179)
- `/lio_sam/imu/path` (imuPreintegration, L75)

**해결 방향:** `utility.hpp`의 `qos_profile` Reliability를 `RELIABLE`로 변경하거나, RViz config에서 해당 토픽 구독 시 `Reliability: Best Effort` 설정

```cpp
// utility.hpp:412 수정 전
RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,

// 수정 후
RMW_QOS_POLICY_RELIABILITY_RELIABLE,
```

**주의:** Reliability를 RELIABLE로 변경하면 네트워크 환경에서 지연이 발생할 수 있으므로, 실제 로봇 환경에서는 BEST_EFFORT가 적합. Gazebo 시뮬레이션용으로만 변경 권장.

---

#### Note 1: Point cloud deskew 비활성화 (심각도: 중간)

**증상:** 기동 직후 경고 발생, 이후 localization pose가 프레임마다 수 미터씩 점프

```
[imageProjection] Point cloud timestamp not available, deskew function disabled, system will drift significantly!
```

**TF echo 결과 (로봇 정지 상태):**
```
Translation: [0.033, 0.319, -0.282]   → (0초)
Translation: [3.433, -1.518, -0.424]  → (2초)
Translation: [1.640, -1.806, -0.467]  → (4초)
Translation: [2.837, -1.738, -0.248]  → (6초)
```

**원인:** Gazebo의 3D LiDAR bridge가 PointCloud2 메시지에 per-point timestamp를 포함하지 않음. LIO-SAM의 deskew(모션 보정)가 비활성화되어 scan-to-map 매칭 품질 저하.

**영향:** Localization 기능적으로는 동작하나, 실용적 수준의 정밀도 확보 불가. SLAM 모드에서도 동일 현상 발생하나 factor graph 최적화가 드리프트를 부분 보정함.

**해결 방향:** Gazebo 시뮬레이션 한계. 실제 LiDAR (Velodyne/Livox)는 per-point timestamp 제공하므로 실환경에서는 문제 없음.
