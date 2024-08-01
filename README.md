# HITBOT Gripper Workspace

This repository contains the setup and launch instructions for working with the HITBOT gripper in a ROS (Robot Operating System) environment. It includes the necessary files to simulate and control the HITBOT gripper in Gazebo, and perform pick-and-place operations.

## Table of Contents
- [Introduction](#introduction)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Introduction
This project sets up a ROS workspace for the HITBOT gripper, integrates it with a UR5 robot, and provides simulation and control functionalities using Gazebo and ROS.

## Installation
Follow these steps to set up the workspace and clone the necessary repositories.

### Step 1: Create and initialize the workspace
Open a terminal and execute the following commands:
```sh
mkdir -p hitbot_gripper_ws/src
cd hitbot_gripper_ws
catkin_make
git clone ~
catkin_make
source devel/setup.bash
roslaunch demo6_config demo_gazebo.launch
roslaunch demo6_config demo.launch
roslaunch UR5_pick_and_place pick_and_place

