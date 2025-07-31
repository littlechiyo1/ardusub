cmake_minimum_required(VERSION 3.0.2)
project(ultrasonic_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package(serial REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ultrasonic_sensor_node src/ultrasonic_sensor_node.cpp)

target_link_libraries(ultrasonic_sensor_node
  ${catkin_LIBRARIES}
  serial
  ${serial_LIBRARIES}
)

add_dependencies(ultrasonic_sensor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})