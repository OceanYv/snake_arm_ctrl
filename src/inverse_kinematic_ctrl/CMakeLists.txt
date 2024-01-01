cmake_minimum_required(VERSION 3.0.2)
project(inverse_kinematic_ctrl)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp serial std_msgs geometry_msgs std_srvs tf visualization_msgs
  pos_kinematic_ctrl snake_arm_msg
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES pos_kinematic_ctrl
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  lib/sisl
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS} 
)

## 添加sisl静态连接库
link_directories(
  lib/sisl
)

# class inverse_kinematic_ctrl
add_library(inverse_kinematic_ctrl
  include/inverse_kinematic_ctrl.h
  src/inverse_kinematic_ctrl.cpp 
)
target_link_libraries(inverse_kinematic_ctrl common_function libsisl.a libsisl_streaming.a
  ${catkin_LIBRARIES}
)

# ROS inverse_kinematic_ctrl_node
add_executable(inverse_kinematic_ctrl_node
    src/inverse_kinematic_ctrl_node.cpp
)
target_link_libraries(inverse_kinematic_ctrl_node inverse_kinematic_ctrl
  ${catkin_LIBRARIES}
)