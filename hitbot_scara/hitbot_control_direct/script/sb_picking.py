#!/usr/bin/env python

import copy
import math
import rospy

import libscrc
import binascii
import roslib
import struct
import numpy as np
from tf_conversions import transformations
from math import pi
import tf
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

from hitbot_scara_control.msg import HitbotCommand, HitbotPoseRaw

from sb_robot_msgs.msg import Fruits
from hitbot_scara_control.msg import HitbotCommand
from hitbot_scara_control.srv import MoveToTargetPoseService
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point


# picking the strawberry
class StrawberryPicking(object):
    def __init__(self) -> None:
        rospy.init_node("strawberry_picking")
        self.namespace = rospy.get_namespace()
        # initialize publishers
        self.init_variables()
        self.init_publishers()
        self.init_subscribers()
        self.init_services()
        rospy.loginfo(rospy.get_name() + " Start")
        self.picking_rate = rospy.Rate(1)
        self.strawberry_picking()

    def init_variables(self):
        self.detected_fruits = []
        self.home_pose = [140, 0, -80, -50]
        self.place_pose = [200, 0, -180, 152]
        self.claw_pose = [0.10, 10.00, 20.00]
        # joint positions
        self.joint_states = [0, 0, 0, 0, 0, 0]
        self.distance_before_target_point = -40

    def init_publishers(self):
        pass

    def init_services(self):
        service_name = "/scara_move_to_target"
        self.move_scara_proxy = rospy.ServiceProxy(
            service_name, MoveToTargetPoseService
        )

    def init_subscribers(self):
        percepted_fruits_topic = "/perception/fruits"
        rospy.Subscriber(percepted_fruits_topic, Fruits, self.get_percepted_fruits)
        joint_states_topic = "/joint_states"
        rospy.Subscriber(joint_states_topic, JointState, self.get_joint_states)
        hitbot_raw_poses_topic = "/hitbot_raw_poses"
        rospy.Subscriber(
            hitbot_raw_poses_topic, HitbotPoseRaw, self.get_hitbot_raw_poses
        )

        self.tf_listener = tf.TransformListener()
        try:
            self.tf_listener.waitForTransform(
                "/ee_link", "/camera_link", rospy.Time(), rospy.Duration(1.0)
            )
            self.tf_listener_ee_link_to_link3 = tf.TransformListener()
            self.tf_listener.waitForTransform(
                "/base_link", "/ee_link", rospy.Time(), rospy.Duration(1.0)
            )
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return

    def get_percepted_fruits(self, fruit_msg):
        detected_fruits = []
        if len(fruit_msg.fruits) > 0:
            for fruit in fruit_msg.fruits:
                detected_fruits.append([fruit.x, fruit.y, fruit.z])

        self.detected_fruits = detected_fruits

    def get_trans_pos(self, x, y, z):
        target_T_source = np.diag([1, 1, 1, 1])
        try:
            (
                camera_link_to_ee_link_trans,
                camera_link_to_ee_link_rot,
            ) = self.tf_listener.lookupTransform(
                "/ee_link", "/camera_link", rospy.Time(0)
            )
            (
                ee_link_to_base_link_trans,
                ee_link_to_base_link_rot,
            ) = self.tf_listener.lookupTransform(
                "/base_link", "/ee_link", rospy.Time(0)
            )

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.loginfo("tf Error")
            return None
        camera_link_to_ee_link_rot_matrix = tf.transformations.quaternion_matrix(
            [
                camera_link_to_ee_link_rot[0],
                camera_link_to_ee_link_rot[1],
                camera_link_to_ee_link_rot[2],
                camera_link_to_ee_link_rot[3],
            ]
        )
        camera_link_to_ee_link_rot_matrix[0, 3] = camera_link_to_ee_link_trans[0]
        camera_link_to_ee_link_rot_matrix[1, 3] = camera_link_to_ee_link_trans[1]
        camera_link_to_ee_link_rot_matrix[2, 3] = camera_link_to_ee_link_trans[2]

        ee_link_to_base_link_rot_matrix = tf.transformations.quaternion_matrix(
            [
                ee_link_to_base_link_rot[0],
                ee_link_to_base_link_rot[1],
                ee_link_to_base_link_rot[2],
                ee_link_to_base_link_rot[3],
            ]
        )

        ee_link_to_base_link_rot_matrix[0, 3] = ee_link_to_base_link_trans[0]
        ee_link_to_base_link_rot_matrix[1, 3] = ee_link_to_base_link_trans[1]
        ee_link_to_base_link_rot_matrix[2, 3] = ee_link_to_base_link_trans[2]
        ee_origin_point_to_ee_link = np.array([[0.0], [0.0], [0.0], [1]])
        ee_origin_point_to_base_link = (
            np.matmul(ee_link_to_base_link_rot_matrix, ee_origin_point_to_ee_link)
            * 1000
        )
        # print("ee_origin_point_to_base_link:", ee_origin_point_to_base_link)

        target_fruits_postion_to_camera_link = np.array([[0.0], [0.0], [0.0], [1]])
        target_fruits_postion_to_camera_link[0] = -y
        target_fruits_postion_to_camera_link[1] = x
        target_fruits_postion_to_camera_link[2] = z
        # print(target_fruits_postion_to_camera_link)
        target_T_source = camera_link_to_ee_link_rot_matrix.copy()
        target_fruits_postion_to_ee_link = (
            np.matmul(target_T_source, target_fruits_postion_to_camera_link) * 1000
        )
        target_fruits_postion_to_base_link = np.matmul(
            ee_link_to_base_link_rot_matrix, target_fruits_postion_to_ee_link
        )
        # print("target_fruits_postion_to_base_link:", target_fruits_postion_to_base_link)
        # print(
        #     "x1:",
        #     "%.4f" % target_fruits_postion_to_ee_link[0],
        #     "y1:",
        #     "%.4f" % target_fruits_postion_to_ee_link[1],
        #     "z1:",
        #     "%.4f" % target_fruits_postion_to_ee_link[2],
        # )
        target_point_to_base_link_relative_movement_value = np.array(
            [[0.0], [0.0], [0.0]]
        )
        target_point_to_base_link_relative_movement_value[0] = (
            target_fruits_postion_to_base_link[0] - ee_origin_point_to_base_link[0]
        )
        target_point_to_base_link_relative_movement_value[1] = (
            target_fruits_postion_to_base_link[1] - ee_origin_point_to_base_link[1]
        )
        target_point_to_base_link_relative_movement_value[2] = (
            target_fruits_postion_to_base_link[2] - ee_origin_point_to_base_link[2]
        )
        print(
            "target_point_to_base_link_relative_movement_value:",
            target_point_to_base_link_relative_movement_value,
        )
        return target_point_to_base_link_relative_movement_value
        # self.publish_fruits_marker(target_fruits_postion_to_ee_link)

    def get_joint_states(self, joint_state_msg):
        z = joint_state_msg.position[0] * 1000  # mm
        angle1 = math.degrees(joint_state_msg.position[1])  # deg
        angle2 = math.degrees(joint_state_msg.position[2])  # deg
        r = math.degrees(joint_state_msg.position[3])  # deg
        self.joint_states = [
            z,
            angle1,
            angle2,
            r,
        ]

    def get_hitbot_raw_poses(self, hitbot_raw_pose_msg):
        z = hitbot_raw_pose_msg.z
        x = hitbot_raw_pose_msg.x
        y = hitbot_raw_pose_msg.y
        self.hitbot_raw_pose = [x, y, z]

    def move_to_pose(self, x, y, z, r, claw_postion, claw_velocity, claw_current):
        hitbot_cmd = HitbotCommand()
        hitbot_cmd.goal_x = x
        hitbot_cmd.goal_y = y
        hitbot_cmd.goal_z = z
        hitbot_cmd.goal_r = r
        hitbot_cmd.moving_speed = 200
        hitbot_cmd.target_speed_ratio = 0
        # the hand type is dependent on the angle2
        if self.joint_states[2] < 0:
            hitbot_cmd.hand_type = -1  # left
        else:
            hitbot_cmd.hand_type = 1  # right
        hitbot_cmd.claw_postion = claw_postion
        hitbot_cmd.claw_velocity = claw_velocity
        hitbot_cmd.claw_current = claw_current
        try:
            response = self.move_scara_proxy(hitbot_cmd)
            if response.success:
                rospy.loginfo(rospy.get_name() + " Hitbot move to the target pose!")
                return True
        except rospy.ServiceException as exc:
            rospy.logerr(
                rospy.get_name() + " Hitbot cannot move to the target pose: " + str(exc)
            )
            return False
        # except rospy.ServiceException as exc:
        #     rospy.logerr(
        #         rospy.get_name() + " Hitbot cannot move to the target pose: " + str(exc)
        #     )
        #     return False

    def e_claw_control(self, operation):
        # open: 1, close:-1
        pass

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

            if len(picking_fruits) == 0:
                for i in range(pos_r, 64, 10):
                    self.move_to_pose(
                        self.home_pose[0],
                        self.home_pose[1],
                        self.home_pose[2],
                        i,
                        self.claw_pose[2],
                        100.00,
                        0.50,
                    )
                    pos_r = i
                    if len(self.detected_fruits) > 0:
                        pos_r = i
                        break
                for i in range(pos_r, -118, -10):
                    self.move_to_pose(
                        self.home_pose[0],
                        self.home_pose[1],
                        self.home_pose[2],
                        i,
                        self.claw_pose[2],
                        100.00,
                        0.50,
                    )
                    pos_r = i
                    if len(self.detected_fruits) > 0:
                        pos_r = i
                        break
            picking_fruits = copy.copy(self.detected_fruits)
            print("detected_fruits", len(picking_fruits))
            if len(self.detected_fruits) > 0:
                # move to the target places and put it back
                for fruit in picking_fruits:
                    x, y, z = fruit[0], fruit[1], fruit[2]
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
                    move_to_target_return = self.move_to_pose(
                        robot_arm_end_current_position[0]
                        + target_point_to_ee_link[0]
                        - self.distance_before_target_point,
                        robot_arm_end_current_position[1] - target_point_to_ee_link[1],
                        robot_arm_end_current_position[2] - target_point_to_ee_link[2],
                        pos_r,
                        self.claw_pose[2],
                        100.00,
                        0.50,
                    )
                    # print("move_to_target_return:", move_to_target_return)
                    print(
                        "x_target:",
                        robot_arm_end_current_position[0] + target_point_to_ee_link[0],
                        "y_target:",
                        robot_arm_end_current_position[1] - target_point_to_ee_link[1],
                        "z_target:",
                        robot_arm_end_current_position[2] - target_point_to_ee_link[2],
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
                            pos_r,
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
    strawberry_picking = StrawberryPicking()
