# Hitbot scara model
Here are the ROS simulation packages for Hitbot(慧灵), You can load robots and joint controllers in Gazebo, as well as in the real robot.

### dependencies
```bash
pip install empy==3.3.4
sudo apt-get update 
sudo apt install libmodbus-dev
sudo apt-get install -y ros-noetic-gazebo-ros-control
sudo apt-get install -y ros-noetic-joint-trajectory-controller ros-noetic-position-controllers ros-noetic-ackermann-steering-controller ros-noetic-diff-drive-controller 
sudo apt-get install -y ros-noetic-velocity-controllers
sudo apt-get install -y ros-noetic-controller-interface  ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers 
sudo apt-get install -y ros-noetic-ddynamic-reconfigure
sudo apt-get install -y python-is-python3
sudo apt-get install -y ros-noetic-moveit
sudo apt install -y ros-noetic-catkin python3-catkin-tools python3-wstool
sudo apt install -y ros-noetic-joy-teleop ros-noetic-spacenav-node ros-noetic-rosparam-shortcuts ros-noetic-joy libqt5x11extras5-dev ros-noetic-graph-msgs ros-noetic-code-coverage ros-noetic-franka-description
sudo apt install -y ros-noetic-pinocchio
sudo apt install -y ros-noetic-trac-ik-kinematics-plugin ros-noetic-rviz-visual-tools ros-noetic-moveit-visual-tools
sudo apt install -y python3-pybind11
sudo apt install -y python3-pip python-is-python3
sudo apt install -y libboost-all-dev libeigen3-dev liburdfdom-dev
sudo apt install -y iputils-ping
sudo apt install -y ros-$ROS_DISTRO-realsense2-camera
```

## Packages:
### `hitbot_model` 
It includes the definitions of hitbot urdf, meshes, and controllers.


### `hitbot_moveit_config` 
Generated from ros_moveit_assistant and edited accordingly.

### `hitbot_control_direct` 
Hitbot is a scara robotic arm whose IK can be expressed with a not very complex function. Hitbot has already integrated its planner inside with easy API. We wrote a ROS controller to manipulate robot in an agonostic way with rosservice.

### `hitbot_hw`
Interface setup for communicating low level motors and ROS controller manager.


### `hitbot_twin_control`
control the hitbot with twin robot arm


## Commands:
 You can test some function through simple commands below.

```bash
# Moveit! + Gazebo + Rviz
roslaunch hitbot_bringup sim_arm.launch 
```

```bash
# Real robot
roslaunch hitbot_bringup real_arm.launch 

```

```bash
# PID test 
roslaunch hitbot_model position_gazebo.launch 
rosrun hitbot_model joint_test_control.py

```

```bash
# run hitbot_twin_robot
rosrun hitbot_twin_control hitbot_twin_control_node
```

