cmake_minimum_required(VERSION 3.0.2)
project(fcu_core)

## ---------------------- 编译设置 ----------------------
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# ✅ 关闭 MAVLink 的结构体对齐警告
add_compile_options(-Wno-address-of-packed-member -Wno-unused-variable -Wno-sign-compare)

## ---------------------- 依赖包 ------------------------
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  message_generation
  image_transport
  tf
  serial
  quadrotor_msgs
)

# ✅ YAML 解析库（用于读取坐标文件）
find_package(yaml-cpp REQUIRED)

# ✅ Eigen3 支持（工程师原本包含）
find_package(Eigen3 REQUIRED)

## ---------------------- 消息生成 ----------------------
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  serial
  quadrotor_msgs
)

## ---------------------- 头文件路径 ----------------------
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## ---------------------- 可执行节点 ----------------------

# UAV bridge 节点
add_executable(fcu_bridge_001 src/fcu_bridge_001.cpp)
target_link_libraries(fcu_bridge_001 ${catkin_LIBRARIES})

add_executable(fcu_bridge_002 src/fcu_bridge_002.cpp)
target_link_libraries(fcu_bridge_002 ${catkin_LIBRARIES})

add_executable(fcu_bridge_003 src/fcu_bridge_003.cpp)
target_link_libraries(fcu_bridge_003 ${catkin_LIBRARIES})

add_executable(fcu_bridge_004 src/fcu_bridge_004.cpp)
target_link_libraries(fcu_bridge_004 ${catkin_LIBRARIES})

add_executable(fcu_bridge_005 src/fcu_bridge_005.cpp)
target_link_libraries(fcu_bridge_005 ${catkin_LIBRARIES})

add_executable(fcu_bridge_006 src/fcu_bridge_006.cpp)
target_link_libraries(fcu_bridge_006 ${catkin_LIBRARIES})

# command 节点
add_executable(fcu_command src/fcu_command.cpp)
target_link_libraries(fcu_command ${catkin_LIBRARIES})

# ✅ mission 节点：支持从 YAML 文件读取坐标
add_executable(fcu_mission src/fcu_mission.cpp)
target_link_libraries(fcu_mission
  ${catkin_LIBRARIES}
  yaml-cpp
)

## ---------------------- 安装与资源拷贝 ----------------------
install(TARGETS
  fcu_bridge_001
  fcu_bridge_002
  fcu_bridge_003
  fcu_bridge_004
  fcu_bridge_005
  fcu_bridge_006
  fcu_command
  fcu_mission
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz_cfg config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

