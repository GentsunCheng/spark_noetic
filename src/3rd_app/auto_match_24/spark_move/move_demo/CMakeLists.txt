cmake_minimum_required(VERSION 2.8.3)
project(move_demo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs  # 添加此行以使得geometry_msgs依赖项生效
  common
  actionlib
  actionlib_msgs
  dynamic_reconfigure
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES move_demo
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs actionlib actionlib_msgs  # 添加geometry_msgs依赖项
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## 安装launch文件（如果有的话）
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## 安装Python脚本，确保它们是可执行的
install(PROGRAMS
  src/teleop_twist_keyboard.py  # 添加你的脚本路径
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)