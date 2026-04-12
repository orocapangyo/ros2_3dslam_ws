# route_graph_builder 실행 명령어

## 빌드

```bash
colcon build --packages-select route_graph_builder odd_costmap_generator
source install/setup.bash
```

## 실행 (RViz2 포함)

```bash
ros2 launch odd_costmap_generator odd_costmap_test.launch.py \
  waypoint_file:=$HOME/Study/ros2_3dslam_ws/waypoints/job_test_large_loop.txt \
  edge_file:=$HOME/Study/ros2_3dslam_ws/waypoints/edges_test_large_loop.txt
```
