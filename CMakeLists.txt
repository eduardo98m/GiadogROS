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
  /usr/include/eigen3
  /src/joystick/c++
)

add_executable(servo_test src/servo_test.cpp)

add_executable(imu_test src/imu_test.cpp)
add_dependencies(imu_test sensor_msgs_generate_messages_cpp)

add_library(joystick_lib src/joystick/c++/joy.cpp)
add_executable(joystick src/joystick.cpp src/joystick/c++/joy.cpp)


add_executable(servo_translate src/servo_translate.cpp)
add_executable(gait src/gait.cpp)
add_dependencies(gait sensor_msgs_generate_messages_cpp)

target_link_libraries(servo_test
  ${catkin_LIBRARIES}
  -lyaml-cpp
)
target_link_libraries(servo_translate
  ${catkin_LIBRARIES}
  -lyaml-cpp
)
target_link_libraries(gait
  ${catkin_LIBRARIES}
  -lyaml-cpp
)
target_link_libraries(imu_test
  ${catkin_LIBRARIES}
)

target_link_libraries(joystick
 ${catkin_LIBRARIES}
 joystick_lib
)


## Install node
install(TARGETS servo_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS imu_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS servo_translate
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS gait
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS joystick
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)