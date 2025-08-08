cmake_minimum_required(VERSION 3.0.2)
project(ultrasonic_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

# 添加本地 serial 库
set(SERIAL_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/serial)
add_subdirectory(${SERIAL_LIB_PATH} ${CMAKE_CURRENT_BINARY_DIR}/serial)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs
  DEPENDS 
    serial
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${SERIAL_LIB_PATH}/include
  src/ultrasonic/include
)

add_executable(ultrasonic_sensor_node
  src/main.cpp
  src/ultrasonic/src/ultrasonic_sensor_node.cpp
)

target_link_libraries(ultrasonic_sensor_node
  ${catkin_LIBRARIES}
  serial
)

add_dependencies(ultrasonic_sensor_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)