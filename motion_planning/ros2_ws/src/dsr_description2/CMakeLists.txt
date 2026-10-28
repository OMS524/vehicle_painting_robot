cmake_minimum_required(VERSION 3.8)
project(dsr_description2)

find_package(ament_cmake REQUIRED)

# package://dsr_description2/meshes/... 를 해결하기 위한 설치.
# RViz/robot_state_publisher 는 ament index 로 share 디렉터리를 찾는다.
get_filename_component(MESH_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/meshes" REALPATH)
install(DIRECTORY "${MESH_SOURCE}/" DESTINATION share/${PROJECT_NAME}/meshes)

ament_package()
