# ACS Waypoint System

## 빌드

```bash
colcon build --packages-select amr_interfaces amr_motion_control_2wd waypoint_interfaces waypoint_manager acs_waypoint_gui
source install/setup.bash
```

> **Note:** `waypoint_interfaces` 는 `waypoint_manager` / `acs_waypoint_gui` 의 의존 패키지이므로 반드시 함께 빌드해야 한다.
> 누락 시 아래 오류 발생:
> `Failed to find .../install/waypoint_interfaces/share/waypoint_interfaces/package.sh`

## Waypoint 파일 위치

```text
~/Study/ros2_3dslam_ws/waypoints/
```

## 실행

```bash
# GUI + 전체 스택 한번에
ros2 launch acs_waypoint_gui acs_gazebo_full.launch.py
```

## 통합 테스트 (자동)

```bash
# 전체
bash src/ACS/Waypoint-System/acs_waypoint_gui/run_integration_tests.sh

# 단일 시나리오
bash src/ACS/Waypoint-System/acs_waypoint_gui/run_integration_tests.sh gazebo
# 시나리오: gazebo | straight | large_loop | multistop | drive_modes | yawctrl | precision
```
