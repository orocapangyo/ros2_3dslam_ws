cmake_minimum_required(VERSION 3.8)
project(nav2_bringup_3dslam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  docs
  rviz2
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
