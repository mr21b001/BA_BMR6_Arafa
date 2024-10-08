cmake_minimum_required(VERSION 3.3)
project(ov_msckf)

# Include libraries (if we don't have OpenCV 4, then fallback to OpenCV 3)
# The OpenCV version needs to match the one used by cv_bridge, otherwise, you will get a segmentation fault!
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 QUIET)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Ceres REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)

include_directories(${catkin_INCLUDE_DIRS})

# Add the OpenVINS node with depth integration
add_executable(openvins_node_with_depth src/openvins_node_with_depth.cpp)

# Link the necessary libraries
target_link_libraries(openvins_node_with_depth
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
    ov_core_lib  # Correctly linking to the available libraries
    ov_msckf_lib
    ov_eval_lib  # Include if needed
    ov_init_lib  # Include if needed
)

message(STATUS "OPENCV: " ${OpenCV_VERSION} " | BOOST: " ${Boost_VERSION} " | CERES: " ${Ceres_VERSION})

# Option to enable or disable ArUco tags (disable if no contrib modules)
option(ENABLE_ARUCO_TAGS "Enable or disable aruco tag (disable if no contrib modules)" ON)
if (NOT ENABLE_ARUCO_TAGS)
    add_definitions(-DENABLE_ARUCO_TAGS=0)
    message(WARNING "DISABLING ARUCOTAG TRACKING!")
else ()
    add_definitions(-DENABLE_ARUCO_TAGS=1)
endif ()

# Set C++ standard to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -fno-omit-frame-pointer")

# Find our ROS version
# Default to using the ROS1 package if both are in the environment
find_package(catkin QUIET COMPONENTS roscpp cv_bridge sensor_msgs std_msgs)
find_package(ament_cmake QUIET)

if (catkin_FOUND)
    message(STATUS "ROS *1* version found, building ROS1.cmake")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
elseif (ament_cmake_FOUND)
    message(STATUS "ROS *2* version found, building ROS2.cmake")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS2.cmake)
else ()
    message(STATUS "No ROS versions found, building ROS1.cmake")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
endif ()

# Specify include directories for open_vins and dependencies
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include  # Assuming openvins_node_with_depth.cpp dependencies are in include/
    ${catkin_INCLUDE_DIRS}  # ROS dependencies
    ${EIGEN3_INCLUDE_DIRS}  # Eigen3
    ${OpenCV_INCLUDE_DIRS}  # OpenCV for image processing
)
