cmake_minimum_required(VERSION 2.8.3)
project(boxbot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
)

add_executable(mimic_node src/mimic.cpp)
target_link_libraries(mimic_node
  ${catkin_LIBRARIES}
)
