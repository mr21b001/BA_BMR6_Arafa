# Sensor-based Local Trajectory Planning for Mobile Robots

This project implements two approaches for the Husky robot using ROS: a traditional approach using ROS navigation stacks and an OpenVins approach for visual-inertial odometry.

## Installation and Setup Guide

This guide explains how to install the necessary dependencies, set up the environment, and start the experiments using both the traditional approach and the OpenVins approach.

### Prerequisites

Ensure your system is updated:
sudo apt-get update && sudo apt-get upgrade

#### Additional ROS Tools

You'll need to install additional tools such as `catkin-tools` and `pcl-ros`:

sudo apt-get install python3-catkin-tools python3-osrf-pycommon  # for Ubuntu 20.04
sudo apt-get install ros-$(rosversion -d)-pcl-ros  # Install PCL package for ROS

Add ROS setup to your bash configuration to source ROS automatically:


### Step 1: Install Dependencies

You'll need several libraries to build and run the solution, such as Eigen, Boost, and Ceres Solver:

sudo apt-get install libeigen3-dev libboost-all-dev libceres-dev

### Step 2: Install OpenCV

To use OpenCV, follow these steps to install the library:

git clone https://github.com/opencv/opencv/
git clone https://github.com/opencv/opencv_contrib/
mkdir opencv/build/
cd opencv/build/
cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
make -j8
sudo make install

### Step 3: Build the ROS Workspace

Make sure you are inside your ROS workspace and then build the solution using `catkin_make`:

cd ~/catkin_ws
catkin_make

### Step 4: Run the Experiments

Once the build is complete, you can run the experiments with two different approaches: the traditional approach and the OpenVins approach.

#### 1) **Traditional Approach**

This uses ROS navigation and simulation packages for the Husky robot:

roslaunch husky_gazebo husky_playpen.launch
roslaunch husky_viz view_robot.launch
roslaunch husky_navigation amcl_demo.launch
rosrun husky_goals_pkg husky_goals

#### 2) **OpenVins Approach**

This utilizes the OpenVins package for visual-inertial odometry with the Husky robot:

roslaunch husky_gazebo husky_playpen.launch
roslaunch ov_msckf simulation.launch  # Wait until after the launch

rosrun husky_goals_pkg husky_openvins

### Step 5: Install Ceres Solver (Optional)

If needed, Ceres Solver can be installed as follows:

sudo apt-get install libceres-dev


### Sources of ROS Packages

The following ROS packages and repositories were used in this solution:

- **Husky Packages (Gazebo, Viz, Navigation)**:
  - `husky_gazebo`, `husky_viz`, , `husky_control`, `husky_description`, `husky_navigation` and `husky_navigation` are part of the Husky robot simulation package. These can be installed via `apt`:
sudo apt-get install ros-noetic-husky-simulator ros-noetic-husky-navigation ros-noetic-husky-gazebo ros-noetic-husky-viz

  - The packages are maintained by Clearpath Robotics and can be found here: https://github.com/husky/husky

- **OpenVins**:
  - OpenVins is an open-source visual-inertial odometry system. The repository can be found here: https://github.com/rpng/open_vins




- **Husky Goals Package**:
  - Custom `husky_goals_pkg` package for setting goals for the Husky robot in both the traditional and OpenVins approaches. This package should be included in your workspace.


By following this guide, you should be able to set up the necessary environment, install all dependencies, and reproduce the experiments as described.
