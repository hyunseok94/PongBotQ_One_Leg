cmake_minimum_required(VERSION 2.8.3)
project(pongbot_leg2_v1)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs 
) 

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/pongbot_leg2_plugin.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
#   INCLUDE_DIRS include
   LIBRARIES pongbot_leg2_v1
   DEPENDS gazebo_ros roscpp rospy std_msgs
#  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


