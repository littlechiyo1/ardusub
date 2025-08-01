cmake_minimum_required(VERSION 3.0.2)
project(ultrasonic_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package(serial REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)

find_library(MOSQPP_LIBRARIES mosquittopp)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
  INCLUDE_DIRS src/ultrasonic/include
  LIBRARIES ultrasonic_sensor_lib
)

include_directories(
  src/ultrasonic/include
  ${catkin_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
  ../rov_planning/src/mqtt/include
  ../rov_planning/src/common
)

add_library(ultrasonic_sensor_lib
  src/ultrasonic/src/ultrasonic_sensor_node.cpp
  ../rov_planning/src/mqtt/src/mqtt.cpp
)

add_executable(ultrasonic_sensor_node
  src/main.cpp
)

target_link_libraries(ultrasonic_sensor_lib
  ${catkin_LIBRARIES}
  ${serial_LIBRARIES}
  ${MOSQUITTO_LIBRARIES}
  ${MOSQPP_LIBRARIES}
)

target_link_libraries(ultrasonic_sensor_node
  ultrasonic_sensor_lib
  ${catkin_LIBRARIES}
)

add_dependencies(ultrasonic_sensor_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ultrasonic_sensor_node ultrasonic_sensor_lib ${catkin_EXPORTED_TARGETS})