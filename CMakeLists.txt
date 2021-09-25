cmake_minimum_required(VERSION 3.0.2)
project(map_switching)

find_package(catkin REQUIRED
    roscpp
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_switching
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(map_selecter src/map_selecter.cpp)
target_link_libraries(map_selecter ${catkin_LIBRARIES})