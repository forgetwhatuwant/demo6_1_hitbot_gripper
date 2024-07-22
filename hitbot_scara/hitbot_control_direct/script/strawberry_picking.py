#!/usr/bin/env python

import copy
import math
import rospy

import libscrc
import binascii
import roslib
import struct

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
        self.home_pose = [200, 0, -80, -28]
        self.place_pose = [200, 0, -180, 152]
        self.claw_pose = [0.10, 10.00, 20.00]

        # joint positions
        self.joint_states = [0, 0, 0, 0]

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

    def get_percepted_fruits(self, fruit_msg):
        detected_fruits = []
        if len(fruit_msg.fruits) > 0:
            for fruit in fruit_msg.fruits:
                detected_fruits.append([fruit.x, fruit.y, fruit.z])

        self.detected_fruits = detected_fruits

    def get_joint_states(self, joint_state_msg):
        z = joint_state_msg.position[0] * 1000  # mm
        angle1 = math.degrees(joint_state_msg.position[1])  # deg
        angle2 = math.degrees(joint_state_msg.position[2])  # deg
        r = math.degrees(joint_state_msg.position[3])  # deg
        x = joint_state_msg.postion[4]
        y = joint_state_msg.postion[5]
        self.joint_states = [z, angle1, angle2, r, x, y]

    def move_to_pose(self, x, y, z, r, claw_postion, claw_velocity, claw_current):
        hitbot_cmd = HitbotCommand()
        hitbot_cmd.goal_x = x
        hitbot_cmd.goal_y = y
        hitbot_cmd.goal_z = z
        hitbot_cmd.goal_r = r
        hitbot_cmd.moving_speed = 20
        hitbot_cmd.target_speed_ratio = 0
        # the hand type is dependent on the angle2
        if self.joint_states[2] < 0:
            hitbot_cmd.hand_type = -1  # left
        else:
            hitbot_cmd.hand_type = 1  # right
        hitbot_cmd.claw_postion = claw_postion
        hitbot_cmd.claw_velocity = claw_velocity
        hitbot_cmd.claw_current = claw_current
        # int8 hand_type
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
            picking_fruits = copy.copy(self.detected_fruits)
            if len(self.detected_fruits) > 0:
                # move to the target places and put it back
                for fruit in picking_fruits:
                    x, y, z = fruit[0], fruit[1], fruit[2]
                    print("x:", x, "y:", y, "z", z)
                    rospy.loginfo(rospy.get_name() + " move to the target pose")
                    # the arm move to the target
                    self.move_to_pose(
                        x,
                        y,
                        z,
                        self.home_pose[3],
                        self.claw_pose[2],
                        100.00,
                        0.50,
                    )
                    rospy.loginfo(
                        rospy.get_name() + " end effect move to the target pose"
                    )
                    # the end effect move to the target
                    ee_angle_move = -(self.joint_states[1] + self.joint_states[2])
                    self.move_to_pose(
                        x, y, z, self.joint_states[3], self.claw_pose[0], 100.00, 0.50
                    )
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
