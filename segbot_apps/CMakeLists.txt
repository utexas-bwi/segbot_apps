cmake_minimum_required(VERSION 2.8.3)
project(segbot_apps)
find_package(catkin REQUIRED COMPONENTS bwi_interruptable_action_server geometry_msgs move_base_msgs nav_msgs std_srvs)
catkin_package(
  CATKIN_DEPENDS bwi_interruptable_action_server geometry_msgs move_base_msgs nav_msgs std_srvs
)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(move_base_interruptable_server src/move_base_interruptable_server.cpp)
target_link_libraries(move_base_interruptable_server
  ${catkin_LIBRARIES}
)
