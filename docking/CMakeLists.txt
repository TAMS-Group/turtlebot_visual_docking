cmake_minimum_required(VERSION 2.8.3)
project(docking)

add_definitions(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  move_base_msgs
  roscpp
  std_msgs
  sensor_msgs
  tf
  diagnostic_msgs	
)

add_action_files(
  DIRECTORY action
  FILES Docking.action GoHome.action
)


generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)



catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES follow_tag
  CATKIN_DEPENDS actionlib actionlib_msgs message_generation move_base_msgs roscpp std_msgs tf diagnostic_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include 
)

add_executable(dock src/dock.cpp src/Docking.cpp src/Avg.cpp)
add_dependencies(dock ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dock ${catkin_LIBRARIES})

add_executable(docking_server src/docking_server.cpp src/Docking.cpp src/Avg.cpp)
add_dependencies(docking_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(docking_server ${catkin_LIBRARIES})
