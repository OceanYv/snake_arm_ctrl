cmake_minimum_required(VERSION 3.0.2)
project(snake_arm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pos_kinematic_ctrl inverse_kinematic_ctrl
  roscpp std_msgs geometry_msgs
  cv_bridge
  image_transport
  snake_arm_msg
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# pcl 
set(PCL_DIR "/usr/local/pcl-1.8/share/pcl-1.8")

find_package(PCL 1.8 REQUIRED COMPONENTS)
link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS
    # include
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
#   include
#   /usr/include/eigen3
  ${catkin_INCLUDE_DIRS} 
)
include_directories(${OpenCV_INCLUDE_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/snake_arm_msg.cpp
# )

add_executable(poses_array_pub
  src/poses_array_pub.cpp
)
target_link_libraries(poses_array_pub
    ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)