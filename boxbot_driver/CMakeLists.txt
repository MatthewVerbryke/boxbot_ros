cmake_minimum_required(VERSION 3.0.2)
project(boxbot_driver)

find_package(catkin REQUIRED 
  COMPONENTS roscpp sensor_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES boxbot_driver
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

add_executable(sim_arbotix_node src/sim_arbotix.cpp)
target_link_libraries(sim_arbotix_node
  ${catkin_LIBRARIES}
  sim_servo
)

add_library(sim_servo src/sim_servo.cpp)
add_library(dynamixel src/dynamixel.cpp)
add_library(interface src/interface.cpp)

add_executable(arbotix_node src/arbotix.cpp)
target_link_libraries(arbotix_node
  ${catkin_LIBRARIES}
  serial
  dynamixel
  interface
)

