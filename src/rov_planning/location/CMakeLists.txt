cmake_minimum_required(VERSION 3.0.2)
project(location)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  sensor_msgs
  geographic_msgs
  nav_msgs
)
find_package(GeographicLib REQUIRED)

# 声明catkin包信息
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp tf2 tf2_ros sensor_msgs geographic_msgs nav_msgs
  DEPENDS GeographicLib
)

# 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

# 生成库文件
add_library(${PROJECT_NAME}
  src/geofence_checker.cpp
  src/gps_postion.cpp
  src/imu_navigator.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

# 生成可执行文件
add_executable(${PROJECT_NAME}_node
  src/location_node.cpp
  main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

# 安装规则（简化版）
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)