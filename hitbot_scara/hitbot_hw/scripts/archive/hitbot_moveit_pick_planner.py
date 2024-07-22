#!/usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg
import random
import sys
import copy
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf
from geometry_msgs.msg import Quaternion
from moveit_commander import RobotCommander, MoveGroupCommander
from sb_robot_msgs.msg import Fruits
import numpy as np
from scipy.spatial.transform import Rotation

# import hitbot_moveit_actioner
import socket
import time
from hitbot_moveit_pick_state_machine import PickingStateMachine

"""
# 1. subscribe the fruit detection result msg
# 2. call the action server to generatet the planned trajectory
"""


class StrawberryPicking(object):
    def __init__(self) -> None:
        rospy.init_node("strawberry_picking")
        self.namespace = rospy.get_namespace()
        # initialize publishers
        # 初始化 ROS 节点
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化 MoveIt Commander
        self.robot = RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "arm_group"
        self.move_group = MoveGroupCommander(self.group_name)
        self.planning_frame = self.move_group.get_planning_frame()
        self.move_group.set_planning_time(5)
        self.init_variables()
        self.init_publishers()
        self.init_subscribers()
        rospy.loginfo(rospy.get_name() + " Start")
        self.picking_rate = rospy.Rate(1)
        self.strawberry_picking()

    def init_variables(self):
        self.detected_fruits = []
        self.home_pose = [140, 0, -80, -28]
        self.place_pose = [200, 0, -180, 152]
        self.claw_pose = [0.10, 10.00, 20.00]
        # joint positions
        self.joint_states = [0, 0, 0, 0, 0, 0]
        self.distance_before_target_point = -40

    def init_publishers(self):
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

    def init_subscribers(self):
        percepted_fruits_topic = "/perception/fruits"
        rospy.Subscriber(percepted_fruits_topic, Fruits, self.get_percepted_fruits)
        # joint_states_topic = "/joint_states"
        # rospy.Subscriber(joint_states_topic, JointState, self.get_joint_states)
        # hitbot_raw_poses_topic = "/hitbot_raw_poses"
        # rospy.Subscriber(
        #     hitbot_raw_poses_topic, HitbotPoseRaw, self.get_hitbot_raw_poses
        # )
        # self.tf_listener = tf.TransformListener()
        # try:
        #     self.tf_listener.waitForTransform(
        #         "/ee_link", "/camera_link", rospy.Time(), rospy.Duration(1.0)
        #     )
        #     self.tf_listener_ee_link_to_link3 = tf.TransformListener()
        #     self.tf_listener.waitForTransform(
        #         "/base_link", "/ee_link", rospy.Time(), rospy.Duration(1.0)
        #     )
        # except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        #     return

    def get_percepted_fruits(self, fruit_msg):
        detected_fruits = []
        if len(fruit_msg.fruits) > 0:
            for fruit in fruit_msg.fruits:
                detected_fruits.append([fruit.x, fruit.y, fruit.z])

        # print("detected_fruits_bbb:", detected_fruits)  # 記得刪
        self.detected_fruits = detected_fruits

    # 歐拉角轉換成四元數
    def euler_to_quaternion(self, roll, pitch, yaw):
        r = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=True)
        quaternion = r.as_quat()
        return quaternion

    # 四元數轉換成歐拉角
    def quaternion_to_euler(self, quaternion):
        r = Rotation.from_quat(quaternion)
        euler = r.as_euler("xyz", degrees=True)
        return euler

    def moveit_plan_to_target(self, x, y, z, yaw, pitch, roll):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        Quaternion = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        # return Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        goal_pose = geometry_msgs.msg.PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = "world"
        goal_pose.pose.position.x = self.x
        goal_pose.pose.position.y = self.y
        goal_pose.pose.position.z = self.z
        goal_pose.pose.orientation.x = Quaternion[0]
        goal_pose.pose.orientation.y = Quaternion[1]
        goal_pose.pose.orientation.z = Quaternion[2]
        goal_pose.pose.orientation.w = Quaternion[3]
        self.move_group.set_pose_target(goal_pose)
        plan_success, plan, planning_time, error_code = self.move_group.plan()
        # print("plan result:", plan)
        self.move_group.execute(plan, wait=False)
        self.move_group.stop()
        # 清除之前的目标
        self.move_group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()

    def strawberry_picking(self):
        # move to the home position
        rospy.loginfo(rospy.get_name() + " move to the home pose")
        self.move_to_pose(
            self.home_pose[0],
            self.home_pose[1],
            self.home_pose[2],
            self.home_pose[3],
            self.claw_pose[2],
            100.00,
            0.50,
        )

        while not rospy.is_shutdown():
            # picking fruits
            pos_r = self.home_pose[3]
            picking_fruits = copy.copy(self.detected_fruits)
            self.detected_fruits.clear()

            if len(picking_fruits) > 0:
                # print("detected_fruits:", picking_fruits)

                # move to the target places and put it back
                for fruit in picking_fruits:
                    x, y, z = fruit[0], fruit[1], fruit[2]
                    print("x:", x)
                    print("y:", y)
                    print("z:", z)
                    picking_fruits.clear()
                    target_point_to_ee_link = self.get_trans_pos(x, y, z)
                    print(
                        "picking_fruits:",
                        z * 1000,
                        -x * 1000,
                        y * 1000,
                    )
                    rospy.loginfo(rospy.get_name() + " move to the target pose")

                    robot_arm_end_current_position = [
                        self.hitbot_raw_pose[0],
                        self.hitbot_raw_pose[1],
                        self.hitbot_raw_pose[2],
                    ]
                    # the arm move to the target

                    if z >= 0.15:  # 只是暫時條件，要改
                        move_to_target_return = self.move_to_pose(
                            robot_arm_end_current_position[0]
                            + target_point_to_ee_link[0]
                            + self.distance_before_target_point,
                            robot_arm_end_current_position[1]
                            - target_point_to_ee_link[1],
                            robot_arm_end_current_position[2]
                            - target_point_to_ee_link[2],
                            self.home_pose[3],
                            self.claw_pose[2],
                            100.00,
                            0.50,
                        )

                        # print("move_to_target_return:", move_to_target_return)
                        print(
                            "x_target:",
                            robot_arm_end_current_position[0]
                            + target_point_to_ee_link[0],
                            "y_target:",
                            robot_arm_end_current_position[1]
                            - target_point_to_ee_link[1],
                            "z_target:",
                            robot_arm_end_current_position[2]
                            - target_point_to_ee_link[2],
                        )
                        rospy.loginfo(
                            rospy.get_name() + " end effect move to the target pose"
                        )
                        # the end effect move to the target
                        if move_to_target_return == True:
                            # print("x:", x, "y:", y, "z", z)
                            # target_point_to_ee_link = self.get_trans_pos(x, y, z)
                            # ee_angle_move = -(self.joint_states[1] + self.joint_states[2])
                            self.move_to_pose(
                                robot_arm_end_current_position[0]
                                + target_point_to_ee_link[0]
                                - 30,
                                robot_arm_end_current_position[1]
                                - target_point_to_ee_link[1],
                                robot_arm_end_current_position[2]
                                - target_point_to_ee_link[2],
                                self.home_pose[3],
                                self.claw_pose[0],
                                100.00,
                                0.50,
                            )
                            print("success")

                            # close claw

                            rospy.loginfo(
                                rospy.get_name() + " end effect move to the place pose"
                            )

                            # put the picked fruit back
                            self.move_to_pose(
                                self.place_pose[0],
                                self.place_pose[1],
                                self.place_pose[2],
                                self.place_pose[3],
                                self.claw_pose[1],
                                100.00,
                                0.50,
                            )
                            # open claw
            picking_fruits.clear()
            rospy.loginfo(rospy.get_name() + " move to the home pose")
            self.move_to_pose(
                self.home_pose[0],
                self.home_pose[1],
                self.home_pose[2],
                self.home_pose[3],
                self.claw_pose[2],
                100.00,
                0.50,
            )

            self.picking_rate.sleep()


if __name__ == "__main__":
    while not rospy.is_shutdown():
        pass
    # try:
    #     # moveit_example()
    # except rospy.ROSInterruptException:
