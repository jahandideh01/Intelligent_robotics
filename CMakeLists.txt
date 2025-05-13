cmake_minimum_required(VERSION 3.0.2)
project(ir2425_group_16)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  move_base_msgs
  actionlib
  apriltag_ros
)

## Generate messages and services
add_message_files(
  FILES
  ApriltagIDs.msg
)

add_service_files(
  FILES
  Coeffs.srv
  GetApriltagIDs.srv

)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime tf2 tf2_ros tf2_geometry_msgs move_base_msgs actionlib apriltag_ros geometry_msgs
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare executables
add_executable(node_a src/node_a.cpp)
add_executable(node_b src/node_b.cpp)
add_executable(coeffs_service_server src/coeffs_service_server.cpp)

## Add dependencies
add_dependencies(node_a ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(node_b ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(coeffs_service_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Link libraries
target_link_libraries(node_a
  ${catkin_LIBRARIES}
)

target_link_libraries(node_b
  ${catkin_LIBRARIES}
)


target_link_libraries(coeffs_service_server
  ${catkin_LIBRARIES}
)

## Install launch directory
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

