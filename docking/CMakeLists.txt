cmake_minimum_required(VERSION 2.8.3)
project(docking)

add_definitions(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  message_generation
  move_base_msgs
  roscpp
  std_msgs
  sensor_msgs
  tf
  diagnostic_msgs	
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES follow_tag
  CATKIN_DEPENDS actionlib message_generation move_base_msgs roscpp std_msgs tf diagnostic_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(stop_distance_node src/stop_distance_node.cpp)
add_dependencies(stop_distance_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(docking_node src/docking_node.cpp)
add_dependencies(docking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(docking_node   ${catkin_LIBRARIES} )
target_link_libraries(stop_distance_node   ${catkin_LIBRARIES} )
