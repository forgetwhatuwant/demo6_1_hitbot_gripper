#!/usr/bin/env python
import rospy
import roslib
import actionlib
import control_msgs.msg
from sensor_msgs.msg import JointState
import sys
import libscrc
import struct
import time
import numpy as np
from hitbot_interface.HitbotInterface import HitbotInterface
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from simple_queue import SimpleQueue
import math
import threading
import time

# from Queue import Queue, LifoQueue, PriorityQueue
# good

"""
# 1. execute the trajectory from the actions
# 2. publish joint states
"""


class Hitbot_Moveit_Actioner_Control:
    SCARA_Z_RANGE = 410
    EE_OFFSET = 28

    def __init__(self):
        self.FollowJointTrajectory = control_msgs.msg.FollowJointTrajectoryAction()
        self.new_trajectory = False

        # action server to process the joint trajectory
        self.server = actionlib.SimpleActionServer(
            "/hitbot_control/follow_joint_trajectory",
            control_msgs.msg.FollowJointTrajectoryAction,
            self.execute,
            False,
        )
        # self.server.preempt_request = True
        abs_lib_path = (
            roslib.packages.get_pkg_dir("hitbot_hw") + "/scripts/hitbot_interface"
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
        self.control_rate = rospy.Rate(100)
        self.joint_pub_rate = rospy.Rate(50)
        self.server.start()
        print("FollowJointTrajectoryActionServer start succeeded\n")
        self.target_joint_positions = SimpleQueue()
        self.target_joint_velocities = SimpleQueue()

        joint_pub_thread = threading.Thread(target=self.joint_publish_thread)
        joint_pub_thread.start()

    def joint_publish_thread(self):
        while not rospy.is_shutdown():
            self.publish_joint_states()
            self.joint_pub_rate.sleep()

    def execute(self, Trajectory):
        # print(goal.trajectory.points)
        self.FollowJointTrajectory = Trajectory

        self.server.set_succeeded()
        self.trajetctory_points = Trajectory.trajectory.points

        # flag of getting new trajectories
        self.new_trajectory = True

    def scara_arm_init(self, robot_id, lib_path):
        # joint encoder readings
        self.z_to_base = 0
        self.x_to_base = 0
        self.y_to_base = 0
        self.angle_ee_to_base = 0

        self.angle1 = 0
        self.angle2 = 0
        self.angle3 = 0
        self.goal_angle1 = 0
        self.goal_angle2 = 0
        self.goal_r = 0
        self.goal_z = 0

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

    def update_trajectory(self):
        if not self.target_joint_positions.is_empty():
            print("the path has not been finished!!!!!")
        self.target_joint_positions.clean()
        self.target_joint_velocities.clean()
        for goal_point in self.trajetctory_points:
            self.target_joint_positions.enqueue(goal_point.positions)
            self.target_joint_velocities.enqueue(goal_point.velocities)

        return

    def move_to_target(self, target_positions, target_speed):
        # hitbot command
        self.goal_angle1 = target_positions[1] * (180 / 3.1415926)
        self.goal_angle2 = target_positions[2] * (180 / 3.1415926)
        self.goal_z = target_positions[0] * 1000
        self.goal_r = (
            target_positions[3] * (180 / 3.1415926)
            + self.goal_angle1
            + self.goal_angle2
            - self.EE_OFFSET
        )
        speed = (
            target_speed[1] * (180 / 3.1415926)
            + target_speed[2] * (180 / 3.1415926)
            + target_speed[3] * (180 / 3.1415926)
        ) / 3

        ret = self.scara_arm.new_movej_angle(
            self.goal_angle1, self.goal_angle2, self.goal_z, self.goal_r, 50, 1
        )
        self.return_logging(ret)

    def return_logging(self, ret):
        if ret == 0:
            rospy.logwarn(
                rospy.get_name()
                + " is executing the other command, invalid commanding!"
            )
        if ret == 1:
            pass
        if ret == 2:
            rospy.logwarn(rospy.get_name() + " Wrong speed is set!")
        if ret == 3:
            rospy.logwarn(rospy.get_name() + " is not initialized")
        if ret == 4:
            rospy.logwarn(
                rospy.get_name() + " cannot reach the target which is unarraivable!"
            )
        if ret == 5:
            rospy.logwarn(rospy.get_name() + " servo motor is off!")
        if ret == 11:
            rospy.logwarn(
                rospy.get_name()
                + " is executing the other command, invalid commanding!"
            )
        if ret == 99:
            rospy.logwarn(rospy.get_name() + " is emergently stopped!")
        if ret == 101:
            rospy.logwarn(rospy.get_name() + " wrong parameters!")
        if ret == 102:
            rospy.logwarn(rospy.get_name() + " collision happens!")
        if ret == 103:
            rospy.logwarn(rospy.get_name() + " axises reset!")


if __name__ == "__main__":
    # ROS节点初始化
    # rospy.init_node("joint_state_publisher", anonymous=True)
    server = Hitbot_Moveit_Actioner_Control()
    updated = 0
    while not rospy.is_shutdown():
        # publish joint states
        # server.publish_joint_states()

        if server.new_trajectory:
            server.update_trajectory()
            server.new_trajectory = False
            updated += 1
            print("updated: ", updated)
            goal_position = server.target_joint_positions.dequeue()
            goal_velocity = server.target_joint_velocities.dequeue()

            server.move_to_target(goal_position, goal_velocity)

        # get the command
        if (
            not server.target_joint_positions.is_empty()
            and server.scara_arm.is_robot_goto_target()
        ):
            goal_position = server.target_joint_positions.dequeue()
            goal_velocity = server.target_joint_velocities.dequeue()

            server.move_to_target(goal_position, goal_velocity)

        server.control_rate.sleep()
