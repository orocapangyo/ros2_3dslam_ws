cmake_minimum_required(VERSION 3.5)
project(cartographer_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
