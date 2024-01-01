cmake_minimum_required(VERSION 3.0.2)
project(snake_arm_visual)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp serial std_msgs geometry_msgs std_srvs tf visualization_msgs
  pos_kinematic_ctrl snake_arm_msg realsense2_camera
  sensor_msgs nav_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# pcl 
set(PCL_DIR "/usr/local/pcl-1.8/share/pcl-1.8")

find_package(PCL 1.8 REQUIRED COMPONENTS)
link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
#  INCLUDE_DIRS include
#  LIBRARIES snake_arm_msg
 CATKIN_DEPENDS geometry_msgs message_generation realsense2_camera roscpp rospy std_msgs message_runtime sensor_msgs
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

# # imu数据转里程计，效果不好，弃用
# add_executable(imu_to_odom
#   src/imu_to_odom.cpp
# )
# target_link_libraries(imu_to_odom
#   ${catkin_LIBRARIES}
# )

# 个性化改动的conditional_euclidean_clustering
add_library(conditional_euclidean_clustering_my
  include/conditional_euclidean_clustering_my.h
  include/conditional_euclidean_clustering_my.hpp
  src/conditional_euclidean_clustering_my.cpp
)
target_link_libraries(conditional_euclidean_clustering_my
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

# class snake_arm_visual
add_library(snake_arm_visual
  include/snake_arm_visual.h
  src/snake_arm_visual.cpp 
)
target_link_libraries(snake_arm_visual common_function 
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} conditional_euclidean_clustering_my
  libsisl.a libsisl_streaming.a
)

# class snake_visual_servo
add_library(snake_visual_servo
  include/snake_visual_servo.h
  src/snake_visual_servo.cpp 
)
target_link_libraries(snake_visual_servo common_function 
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

# ROS snake_arm_visual_node
add_executable(snake_arm_visual_node
    src/snake_arm_visual_node.cpp
)
target_link_libraries(snake_arm_visual_node snake_arm_visual snake_visual_servo
  ${catkin_LIBRARIES}
)