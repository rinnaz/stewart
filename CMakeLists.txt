cmake_minimum_required(VERSION 3.0.2)
project(stewart)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
SET(Eigen3_INCLUDE_DIRS "/usr/local/include/eigen3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cmake_modules
)

find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(DEPENDS Eigen3)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

add_executable(ps4_controller src/ps4_controller.cpp)
target_link_libraries(ps4_controller ${catkin_LIBRARIES})

add_executable(ik src/ik.cpp)
target_link_libraries(ik ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(joint_controller SHARED plugin/joint_controller.cc)
target_link_libraries(joint_controller ${GAZEBO_libraries} ${roscpp_LIBRARIES})