cmake_minimum_required(VERSION 3.8)
project(rtab_map_3d_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
