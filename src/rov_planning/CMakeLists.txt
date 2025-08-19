cmake_minimum_required(VERSION 3.0.2)
project(rov_planning)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  mavros_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs mavros_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED libmosquittopp)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src/application/include
  src/mqtt/include
  src/common
  src/ros_com/include
  src/driver/include
  /home/nvidia/catkin_ws/src/geometry2/tf2/include
  ${MOSQUITTO_INCLUDE_DIRS}
)

file(GLOB_RECURSE SOURCES
src/*.cpp
)

add_executable(${PROJECT_NAME}_node 
  ${SOURCES})

### 添加PD控制器库
add_library(pd_controller
  src/pd_controller/src/pd_controller.cpp
)
target_include_directories(pd_controller PUBLIC
  src/pd_controller/include
)
target_link_libraries(pd_controller
  ${catkin_LIBRARIES}
)
###

target_link_libraries(${PROJECT_NAME}_node
  # pdcontroller
  pd_controller
  ${catkin_LIBRARIES}
  ${MOSQUITTO_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/openssl_install/lib64/libcrypto.so.3
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/openssl_install/lib64/libssl.so.3
)