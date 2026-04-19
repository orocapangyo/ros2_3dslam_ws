# Nav2 사용법

## 실행

```bash
# AMR 모션 제어 종료 (cmd_vel 경쟁 방지)
pkill -f amr_motion_control_2wd

# 원샷 (Gazebo + RTAB-Map + Nav2 + RViz)
ros2 launch nav2_bringup_3dslam nav2_full_bringup.launch.py

# 또는 Gazebo/RTAB-Map 이미 실행 중이면
ros2 launch nav2_bringup_3dslam nav2_only.launch.py
```

## 목적지 전송

RViz에서 `2D Goal Pose` 클릭, 또는 CLI:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}"
```
