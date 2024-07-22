#!/usr/bin/env python
import rospy
import roslib
import actionlib
import sensor_msgs.msg
import control_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from math import sin, cos
import sys
import struct
import time
import numpy as np
from hitbot_interface.HitbotInterface import HitbotInterface
from hitbot_scara_moviet_test.msg import HitbotCommand, HitbotPoseRaw
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import math
import tf

# from Queue import Queue, LifoQueue, PriorityQueue


"""
# 1. execute the trajectory from the actions
# 2. publish joint states
"""


class Hitbot_Moveit_Actioner_Control:
    SCARA_Z_RANGE = 410
    EE_OFFSET = 28

    def __init__(self):
        self.FollowJointTrajectory = control_msgs.msg.FollowJointTrajectoryAction()
        self.RecSucceeded = False
        self.server = actionlib.SimpleActionServer(
            "/hitbot_control/follow_joint_trajectory",
            control_msgs.msg.FollowJointTrajectoryAction,
            self.execute,
            False,
        )
        abs_lib_path = (
            roslib.packages.get_pkg_dir("hitbot_scara_moviet_test")
            + "/scripts/hitbot_interface"
        )
        sys.path.append(abs_lib_path)
        # product id
        robot_id = 190
        rospy.init_node("robot_control")
        self.namespace = rospy.get_namespace()
        #
        # initialize the HitbotInterface
        self.scara_arm_init(robot_id, abs_lib_path)
        # initialize publishers
        self.init_publishers()
        # self.init_subscribers()
        # self.init_services()
        # self.listener = tf.listener()
        rospy.loginfo(rospy.get_name() + " Start")
        self.control_rate = rospy.Rate(50)
        self.server.start()
        print("FollowJointTrajectoryActionServer start succeeded\n")

    def execute(self, Trajectory):
        # print(goal.trajectory.points)
        self.FollowJointTrajectory = Trajectory
        self.RecSucceeded = True
        self.server.set_succeeded()
        # self.target_joint_pub(Trajectory.trajectory.points)
        # print(
        #     "self.FollowJointTrajectory.trajectory.points[0]:",
        #     self.FollowJointTrajectory.trajectory.points[0],
        # )
        i = 0
        self.target_joint_positions = []
        self.target_joint_velocities = []
        for goal_joint in self.FollowJointTrajectory.trajectory.points:
            # self.move_to_target(goal_joint.positions, goal_joint.velocities)
            self.target_joint_positions.append(goal_joint.positions)
            self.target_joint_velocities.append(goal_joint.velocities)
        # self.RecSucceeded = False
        # print("target_joint_positions[1]", self.target_joint_positions[1])
        # self.move_to_target(goal_joint.positions, goal_joint.velocities)

    def scara_arm_init(self, robot_id, lib_path):
        # joint encoder readings
        self.z_to_base = 0
        self.x_to_base = 0
        self.y_to_base = 0
        self.angle_ee_to_base = 0

        self.angle1 = 0
        self.angle2 = 0
        self.angle3 = 0

        self.scara_arm = HitbotInterface(robot_id)
        initialized = self.scara_arm.net_port_initial(lib_path)
        # claw.com485 initial
        self.scara_arm.com485_initial(115200)
        if not initialized:
            rospy.logerr(rospy.get_name() + " The network connection is wrong!!")

        # initialize the arm
        initialized = self.scara_arm.initial(1, self.SCARA_Z_RANGE)
        if not initialized:
            rospy.logerr(rospy.get_name() + " The arm cannot be initialized!!")

    def init_publishers(self):
        #
        # self.target_joint_pub = rospy.Publisher(
        #     "/move_group/follow_joint_trajectory", JointTrajectory, queue_size=10
        # )
        joint_states_topic = "joint_states"
        self.joint_states_pub = rospy.Publisher(
            joint_states_topic, JointState, queue_size=10
        )

    def publish_joint_states(self):
        # init = time.time()
        self.get_joint_positions()
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
        ]
        joint_state.position = [
            self.z_to_base / 1000,
            math.radians(self.angle1),
            math.radians(self.angle2),
            math.radians(self.angle3),
        ]
        # print("joint_state.position", joint_state.position)

        self.joint_states_pub.publish(joint_state)
        # duration = time.time() - init
        # print("duration: ", duration)

    def get_joint_positions(self):
        self.scara_arm.get_encoder_coor()
        # position relative to base
        self.z_to_base = self.scara_arm.encoder_z
        self.x_to_base = self.scara_arm.encoder_x
        self.y_to_base = self.scara_arm.encoder_y
        self.angle_ee_to_base = self.scara_arm.encoder_r

        # position relative to last link
        self.angle1 = self.scara_arm.encoder_angle1
        self.angle2 = self.scara_arm.encoder_angle2
        self.angle3 = self.angle_ee_to_base - self.angle1 - self.angle2 + self.EE_OFFSET
        # print("encoder_x:", self.scara_arm.encoder_x)
        # print("encoder_y:", self.scara_arm.encoder_y)

    def move_to_target(self, target_positions, target_speed):
        # hitbot command
        goal_angle1 = target_positions[1] * (180 / 3.1415926)
        goal_angle2 = target_positions[2] * (180 / 3.1415926)
        goal_z = target_positions[0] * 1000
        goal_r = (
            target_positions[3] * (180 / 3.1415926)
            + goal_angle1
            + goal_angle2
            - self.EE_OFFSET
        )

        speed = (
            target_speed[1] * (180 / 3.1415926)
            + target_speed[2] * (180 / 3.1415926)
            + target_speed[3] * (180 / 3.1415926)
        ) / 3
        # while self.scara_arm.move_flag != True:
        #     print("self.scara_arm.move_flag:", self.scara_arm.move_flag)
        # while self.scara_arm.move_flag == F:
        self.get_joint_positions()
        self.publish_joint_states()
        print(
            "goal_angle1:",
            goal_angle1,
            "self.angle1:",
            self.scara_arm.encoder_angle1,
        )
        print("self.scara_arm.move_flag:", self.scara_arm.move_flag)
        init = time.time()
        # if goal_angle1 - 4 < self.scara_arm.encoder_angle1 < goal_angle1 + 4:
        ret = self.scara_arm.movej_angle(
            goal_angle1, goal_angle2, goal_z, goal_r, 100, 1
        )
        self.scara_arm.wait_stop()
        # while goal_angle1 - 4 < self.scara_arm.encoder_angle1 < goal_angle1 + 4:
        #     ret = self.scara_arm.movej_angle(
        #         goal_angle1, goal_angle2, goal_z, goal_r, 100, 0.8
        #     )
        print("ret:", ret)

        # while int(self.angle1) != int(goal_angle1):
        #     print("self.angel1:", self.angle1)
        #     self.scara_arm.movej_angle(goal_angle1, goal_angle2, goal_z, goal_r, 50, 1)
        #     self.get_joint_positions()
        duration = time.time() - init
        # print("duration: ", duration)
        # # self.scara_arm.wait_stop()
        # # print("ret:", ret)
        # print(
        #     "goal_angle1:",
        #     goal_angle1,
        #     "goal_angle2:",
        #     goal_angle2,
        #     "goal_z :",
        #     goal_z,
        #     "goal_r :",
        #     goal_r,
        # )

    def return_logging(self, scara_command, ret):
        if ret == scara_command.INVALID:
            rospy.logwarn(
                rospy.get_name()
                + " is executing the other command, invalid commanding!"
            )
        if ret == scara_command.VALID:
            rospy.loginfo(rospy.get_name() + " is executing the command!")
        if ret == scara_command.SPEED_VALID:
            rospy.logwarn(rospy.get_name() + " Wrong speed is set!")
        if ret == scara_command.UNINITED:
            rospy.logwarn(rospy.get_name() + " is not initialized")
        if ret == scara_command.UNARRIVABLE:
            rospy.logwarn(
                rospy.get_name() + " cannot reach the target which is unarraivable!"
            )
        if ret == scara_command.SERVOOFF:
            rospy.logwarn(rospy.get_name() + " servo motor is off!")
        if ret == scara_command.WRONGHANDTYPE:
            rospy.logwarn(rospy.get_name() + " cannot reach with the set hand type!")
        if ret == scara_command.EMERGENCYSTOP:
            rospy.logwarn(
                rospy.get_name()
                + " is executing the other command, invalid commanding!"
            )
        if ret == scara_command.INVALID:
            rospy.logwarn(rospy.get_name() + " is emergently stopped!")
        if ret == scara_command.WRONGPARAMETER:
            rospy.logwarn(rospy.get_name() + " wrong parameters!")
        if ret == scara_command.COLLISION:
            rospy.logwarn(rospy.get_name() + " collision happens!")
        if ret == scara_command.AXISRESET:
            rospy.logwarn(rospy.get_name() + " axises reset!")


if __name__ == "__main__":
    try:
        # ROS节点初始化
        # rospy.init_node("joint_state_publisher", anonymous=True)
        server = Hitbot_Moveit_Actioner_Control()
        while not rospy.is_shutdown():
            # publish joint states
            server.publish_joint_states()
            if server.RecSucceeded:
                print("server.RecSucceeded:", server.RecSucceeded)
                # server.scara_arm.resume_move()
                server.RecSucceeded = False
                # print(
                #     "server.FollowJointTrajectory.trajectory",
                #     server.FollowJointTrajectory.trajectory,
                # )
                # print(server.FollowJointTrajectory.trajectory.points)
                # server.FollowJointTrajectory.trajectory.points: pop the first one
                for goal_joint in server.FollowJointTrajectory.trajectory.points:
                    # server.publish_joint_states()
                    print("server.RecSucceeded:", server.RecSucceeded)
                    # print("server.RecSucceeded", server.RecSucceeded)
                    # if server.RecSucceeded:
                    #     # server.scara_arm.pause_move()
                    #     print("break")

                    #     break
                    # print(goal_joint)
                    # init = time.time()
                    server.move_to_target(goal_joint.positions, goal_joint.velocities)
                    while not server.scara_arm.is_robot_goto_target():
                        server.control_rate.sleep()
                        server.publish_joint_states()

                    # duration = time.time() - init
                    # print("duration: ", duration)
                # rospy.sleep(0.01)
                # print(server.FollowJointTrajectory.trajectory.points.positions)
                # for goal_joint in server.FollowJointTrajectory.trajectory.points:
                # t = rospy.get_rostime()
                # joint_states_msg.header.stamp.secs = t.secs
                # joint_states_msg.header.stamp.nsecs = t.nsecs
                # joint_states_msg.position = goal_joint.positions
                # joint_states_msg.velocity = goal_joint.velocities
                # joint_states_msg.effort = goal_joint.effort
                # # 发布消息
                # joint_states_pub.publish(joint_states_msg)
                # print(goal_joint)
                # rospy.sleep(0.1)
                # joint_state_pub2.publish(joint_states_msg)
                # rate.sleep()

            # print(
            #     hitbot.lookup_transformation("/camera_link", "/base_link", sleep_time=0.5)
            # )
            server.control_rate.sleep()
        # joint_states_publisher()
    except rospy.ROSInterruptException:
        pass
