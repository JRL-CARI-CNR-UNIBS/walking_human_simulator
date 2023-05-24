cmake_minimum_required(VERSION 3.0.2)
project(human_simulator)

add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  roscpp
  actionlib_msgs
  message_generation
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES HumanSimulator.action
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs actionlib_msgs std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES human_simulator
 CATKIN_DEPENDS actionlib_msgs message_runtime std_msgs geometry_msgs moveit_ros_planning_interface roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(human_simulator src/human_simulator.cpp)
add_dependencies(human_simulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(human_simulator
${catkin_LIBRARIES}
)
