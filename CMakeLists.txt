cmake_minimum_required(VERSION 2.8.3)
project(scout_service)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  rospy
  tf
  costmap_2d
  message_generation
  geometry_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  FILES
  POIVector.msg
  RoverPosition.msg
  RoverList.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES scout_service
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(scout_map src/scout_map.cpp)
target_link_libraries(scout_map ${catkin_LIBRARIES})
add_dependencies(scout_map ${catkin_EXPORTED_TARGETS})

add_executable(scout_robot src/scout_robot.cpp)
target_link_libraries(scout_robot ${catkin_LIBRARIES})
add_dependencies(scout_robot ${catkin_EXPORTED_TARGETS})

add_executable(rover_list_manager src/rover_list_manager.cpp)
target_link_libraries(rover_list_manager ${catkin_LIBRARIES})
add_dependencies(rover_list_manager ${catkin_EXPORTED_TARGETS})

add_executable(service_robot src/service_robot.cpp)
target_link_libraries(service_robot ${catkin_LIBRARIES})
add_dependencies(service_robot ${catkin_EXPORTED_TARGETS})
