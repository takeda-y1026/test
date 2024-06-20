cmake_minimum_required(VERSION 3.0.2)
project(demopro)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  sensor_msgs
  tf2
  tf2_ros

  laser_geometry
  tf2_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs

  sound_play
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demopro
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rotate src/rotate.cpp)
add_executable(straight_move src/straight_move.cpp)
add_executable(straight_imp src/straight_imp.cpp)
add_executable(wiper src/wiper.cpp)
add_executable(wiper_imp src/wiper_imp.cpp)
add_executable(number_input src/number_input.cpp)

target_link_libraries(rotate ${catkin_LIBRARIES})
target_link_libraries(straight_move ${catkin_LIBRARIES})
target_link_libraries(straight_imp ${catkin_LIBRARIES})
target_link_libraries(wiper ${catkin_LIBRARIES})
target_link_libraries(wiper_imp ${catkin_LIBRARIES})
target_link_libraries(number_input ${catkin_LIBRARIES})

add_executable(daruma_count src/daruma_count.cpp) #追加
target_link_libraries(daruma_count ${catkin_LIBRARIES})