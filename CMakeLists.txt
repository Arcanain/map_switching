cmake_minimum_required(VERSION 3.0.2)
project(map_switching)

find_package(catkin REQUIRED
    roscpp
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    tf
    visualization_msgs
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

add_executable(map_switching src/map_switching.cpp)
target_link_libraries(map_switching ${catkin_LIBRARIES})

add_executable(simple_map_switch src/simple_map_switch.cpp)
target_link_libraries(simple_map_switch ${catkin_LIBRARIES})

add_executable(Test src/Test.cpp)
target_link_libraries(Test ${catkin_LIBRARIES})

add_executable(map_to_baselink_tf src/map_to_baselink_tf.cpp)
target_link_libraries(map_to_baselink_tf ${catkin_LIBRARIES})