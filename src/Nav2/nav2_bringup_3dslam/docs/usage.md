# Nav2 Bringup 3DSLAM — 사용법

Pioneer2dx + Gazebo + RTAB-Map 3D localization 위에서 Nav2로 목적지 주행.

## 구성

| 컴포넌트 | 플러그인 |
|---|---|
| Global planner | `nav2_smac_planner/SmacPlannerHybrid` (Hybrid-A*, Dubins) |
| Local planner (controller) | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` (RPP) |
| Localization | RTAB-Map 3D LiDAR only (사전 맵 DB 필요) |
| Map source | `/rtabmap/map` (OccupancyGrid, 33.1m×17.6m @ 0.1m/cell) |
| Scan source | `/scan` (Ignition gpu_lidar, 14Hz) |

## 사전 조건

- ROS2 Humble + Nav2 설치
- 워크스페이스 빌드:
  ```bash
  cd ~/Study/ros2_3dslam_ws
  colcon build --packages-select nav2_smac_hybrid nav2_bringup_3dslam --symlink-install
  source install/setup.bash
  ```
- RTAB-Map DB 존재: `maps/rtabmap_3d/rtabmap_3dlidar_only.db`
- AMR motion control이 떠 있다면 **미리 종료** (`/cmd_vel` 경쟁 방지):
  ```bash
  pkill -f amr_motion_control_2wd
  ```

## 실행 방식 A — 원샷 (권장)

```bash
ros2 launch nav2_bringup_3dslam nav2_full_bringup.launch.py
```

Gazebo + RTAB-Map + Nav2 모두 한 번에 기동. Nav2 전용 RViz 자동으로 뜸 (`rviz2/nav2.rviz`).

RViz에 표시되는 경로:
- `/plan` — global planner 출력 (녹색, 두께 0.05)
- `/plan_smoothed` — smoother 적용 후 (청록색, 두께 0.04)
- `/received_global_plan` — RPP controller가 추종 중인 경로 (주황색, 두께 0.06)

RTAB-Map RViz는 기동하지 않음 (`rviz:=false` 전달).

## 실행 방식 B — 단계별

```bash
# 터미널 1: Gazebo (odom_tf 반드시 false)
ros2 launch tm_gazebo gazebo.launch.py odom_tf:=false

# 터미널 2: RTAB-Map localization
ros2 launch rtab_map_3d_config rtabmap_3dlidar_only_localization_gazebo.launch.py

# 터미널 3: Nav2만
ros2 launch nav2_bringup_3dslam nav2_only.launch.py
```

## 목적지 전송

### RViz GUI
`2D Goal Pose` 툴 클릭 → 지도 위에서 드래그로 위치·방향 지정.

### CLI
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'},
   pose: {position: {x: 2.0, y: 0.0, z: 0.0},
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" --feedback
```

## 동작 확인 커맨드

```bash
# 1) Nav2 lifecycle 활성 확인
for n in planner_server smoother_server controller_server behavior_server \
         bt_navigator waypoint_follower velocity_smoother; do
  echo -n "$n: "; ros2 lifecycle get /$n
done
# 기대: 모두 "active [3]"

# 2) 맵 수신 확인
ros2 topic echo --once /rtabmap/map | grep -A1 "width\|height\|resolution"

# 3) 로봇 현재 pose (map frame)
ros2 topic echo --once /rtabmap/localization_pose --qos-reliability best_effort | head -12
```

## 검증된 값 (2026-04-19)

- Start pose (map frame): (0, 0)
- Goal: (2.0, 0.0)
- Reached: (1.78, 0.00) — 허용오차 0.25m 내 도달
- Goal status: **SUCCEEDED**

## 주요 파라미터 위치

- `src/Planner/Nav2/nav2_smac_hybrid/config/smac_hybrid_params.yaml`
  - `local_costmap.global_frame: odom_rtabmap`  ← RTAB-Map의 odom TF 프레임
  - `behavior_server.global_frame: odom_rtabmap`  ← 같은 이유
  - `global_costmap.static_layer.map_topic: /rtabmap/map`
  - `controller_server.FollowPath.desired_linear_vel: 0.5`

## 제한 사항

- `use_sim_time: true` 전제 (실로봇 적용 시 false로 수정 필요)
- Dubins 모델이라 후진 경로 생성 불가 — `allow_reversing: true` + `motion_model_for_search: "REEDS_SHEPP"`로 변경 시 후진 가능
- AMR motion control과 동시 운용 불가 (`/cmd_vel` 경쟁)
