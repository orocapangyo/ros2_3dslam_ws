# slam_manager_3d 실행 명령어

## 빌드
```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select slam_manager_3d --symlink-install
```

## 실행
```bash
source ~/Study/ros2_3dslam_ws/install/setup.bash
ros2 run slam_manager_3d slam_manager_3d
```
