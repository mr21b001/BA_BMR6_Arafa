cmake_minimum_required(VERSION 3.0.2)
project(husky_goals_pkg)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf
  tf2
  tf2_ros
)


find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
	CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs cv_bridge tf)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare the C++ executable husky_goals
add_executable(husky_goals src/husky_goals.cpp)

## Declare the C++ executable husky_openvins
add_executable(husky_openvins src/husky_openvins.cpp)

## Specify libraries to link against for husky_goals
target_link_libraries(husky_goals
  ${catkin_LIBRARIES}
)


## Specify libraries to link against for husky_openvins
target_link_libraries(husky_openvins
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## Add dependencies for husky_goals executable
add_dependencies(husky_goals ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add dependencies for husky_openvins executable
add_dependencies(husky_openvins ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS husky_goals
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS husky_openvins
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
