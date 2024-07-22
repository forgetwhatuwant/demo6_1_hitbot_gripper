cmake_minimum_required(VERSION 3.0.2)
project(hitbot_hw)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  control_msgs
  actionlib_msgs
  sensor_msgs
  actionlib
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
  CATKIN_DEPENDS 
    controller_manager 
    hardware_interface 
    roscpp 
    control_msgs 
    sensor_msgs
    actionlib_msgs
)

###########
## Build ##
###########
find_library(hitbot_interface NAMES hitbot_interface 
             PATHS ${PROJECT_SOURCE_DIR}/extern NO_DEFAULT_PATH)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/extern/include
  ${PROJECT_SOURCE_DIR}/include
)

## Declare a C++ library
set(hitbot_interface ${PROJECT_SOURCE_DIR}/extern/libsmall_scara_interface.so)

add_executable(${PROJECT_NAME}_node src/hitbot_hw.cpp)         
target_include_directories(${PROJECT_NAME}_node PRIVATE /usr/include/modbus)
target_link_libraries(${PROJECT_NAME}_node modbus ${catkin_LIBRARIES} 
                                           ${hitbot_interface}) 

add_executable(hitbot_scara_msg_control_node src/hitbot_hw_msg_control.cpp)         
target_include_directories(hitbot_scara_msg_control_node PRIVATE /usr/include/modbus)
target_link_libraries(hitbot_scara_msg_control_node modbus ${catkin_LIBRARIES} 
                                            ${hitbot_interface}) 


add_executable(gripper_control_node src/gripper_control.cpp)         
target_include_directories(gripper_control_node PRIVATE /usr/include/modbus)
target_link_libraries(gripper_control_node modbus ${catkin_LIBRARIES}) 
