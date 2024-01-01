cmake_minimum_required(VERSION 3.0.2)
project(snake_base_ctrl)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp rospy tf std_msgs serial std_srvs snake_arm_msg
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS thread)

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
    CATKIN_DEPENDS roscpp rospy std_msgs std_srvs serial
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
)

# class snake_base_ctrl
add_library(snake_base_ctrl
  include/snake_base_ctrl.h
  src/snake_base_ctrl.cpp 
)
target_link_libraries(snake_base_ctrl
  ${catkin_LIBRARIES}
)

# ROS snake_base_ctrl_node
add_executable(snake_base_ctrl_node
    src/snake_base_ctrl_node.cpp
)
target_link_libraries(snake_base_ctrl_node snake_base_ctrl
  ${catkin_LIBRARIES}
)