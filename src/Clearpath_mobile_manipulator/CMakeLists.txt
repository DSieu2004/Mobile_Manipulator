cmake_minimum_required(VERSION 3.8)
project(husky_ur5e_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz urdf models mujoco ocs2
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
