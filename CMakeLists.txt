cmake_minimum_required(VERSION 3.0.2)
project(giadog)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  i2cpwm_board
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp i2cpwm_board
)

## Build node
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(servo_test src/servo_test.cpp)

target_link_libraries(servo_test
  ${catkin_LIBRARIES}
)

## Install node
install(TARGETS servo_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)