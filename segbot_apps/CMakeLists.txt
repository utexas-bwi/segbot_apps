cmake_minimum_required(VERSION 2.8.3)
project(segbot_apps)
find_package(catkin REQUIRED COMPONENTS bwi_interruptable_action_server move_base_msgs)
catkin_package(
  CATKIN_DEPENDS bwi_interruptable_action_server move_base_msgs
)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(move_base_interruptable_server src/move_base_interruptable_server.cpp)
target_link_libraries(move_base_interruptable_server
  ${catkin_LIBRARIES}
)
