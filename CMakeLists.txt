cmake_minimum_required(VERSION 3.0.2)
project(elmo_pkgs)


find_package(catkin REQUIRED COMPONENTS 
 roscpp
 std_msgs
 soem
 message_generation
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES elmo_pkgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")

add_executable(elmo_pkgs.exe src/main.cpp src/servo.c src/ecat_dc.c)
target_link_libraries(elmo_pkgs.exe ${catkin_LIBRARIES})


