cmake_minimum_required(VERSION 3.0.2)
project(mapping_node)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找ROS依赖包（添加sensor_msgs）
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  sensor_msgs  # 新增：用于sensor_msgs::NavSatFix
)

# 查找GeographicLib依赖
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

# 声明catkin包信息（添加sensor_msgs依赖）
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    std_srvs 
    geometry_msgs 
    nav_msgs
    tf2
    tf2_ros
    sensor_msgs  
)

# 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}  # GeographicLib头文件
)

# 添加可执行文件（包含gps_position.cpp）
add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/mapping_node.cpp
  src/gps_position.cpp  # GPS_Position类的实现

)

# 链接库文件（添加GeographicLib库）
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}  # 链接GeographicLib

)

# 设置安装规则
install(TARGETS ${PROJECT_NAME}_node

  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)