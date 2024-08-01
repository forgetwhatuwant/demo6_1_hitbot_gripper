mkdir -p hitbot_gripper_ws/src
cd hitbot_gripper
catkin_make
cd src
git clone https://github.com/forgetwhatuwant/demo6_1_hitbot_gripper.git
catkin_make

roslaunch demo6_config demo_gazebo.launch
roslaunch demo6_config demo.launch
roslaunch ur5_pick_and_place pick_and_place
