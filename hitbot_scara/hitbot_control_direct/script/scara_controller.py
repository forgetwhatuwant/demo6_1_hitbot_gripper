#!/usr/bin/env python
import time
import signal
import rospy
import sys
import os
import codecs
import libscrc
import binascii
import roslib
import struct
import time
import numpy as np
from hitbot_interface.HitbotInterface import HitbotInterface
from sensor_msgs.msg import JointState
import math
from hitbot_scara_control.msg import HitbotCommand, HitbotPoseRaw
from hitbot_scara_control.srv import (
    MoveToTargetPoseService,
    MoveToTargetPoseServiceResponse,
)
from shapely import box, LineString, normalize, Polygon, Point
from shapely.geometry import Point
from shapely import affinity
import os
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point
import numpy as np
import tf

# from shapely import Point
import math


class ScaraController(object):
    SCARA_Z_RANGE = 410
    EE_OFFSET = 28

    def __init__(self, robot_id, lib_path):
        rospy.init_node("robot_control")
        self.namespace = rospy.get_namespace()
        #
        # initialize the HitbotInterface
        self.scara_arm_init(robot_id, lib_path)

        # initialize publishers
        self.init_publishers()
        self.init_subscribers()
        self.init_services()
        # self.listener = tf.listener()
        rospy.loginfo(rospy.get_name() + " Start")
        self.control_rate = rospy.Rate(50)

    def init_publishers(self):
        joint_states_topic = "/joint_states"
        self.joint_states_pub = rospy.Publisher(
            joint_states_topic, JointState, queue_size=10
        )
        hitbot_raw_poses_topic = "/hitbot_raw_poses"
        self.hitbot_raw_poses_pub = rospy.Publisher(
            hitbot_raw_poses_topic, HitbotPoseRaw, queue_size=10
        )

    def init_subscribers(self):
        joint_states_topic = "/scara_control"
        rospy.Subscriber(joint_states_topic, HitbotCommand, self.scara_control_execute)

    def init_services(self):
        rospy.Service(
            "/scara_move_to_target",
            MoveToTargetPoseService,
            self.move_to_target_service,
        )

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

    # def lookup_transformation(self, target_frame, source_frame, sleep_time=0.5):
    #     """
    #     In:
    #         target_frame
    #         source_frame
    #     return:
    #         target_T_source
    #     """
    #     success = False
    #     target_T_source = np.diag([1, 1, 1, 1])
    #     while not rospy.is_shutdown() and not success:
    #         try:
    #             (trans, rot) = self.listener.lookupTransform(
    #                 target_frame, source_frame, rospy.Time(0)
    #             )
    #             rot_matrix = tf.transformations.quaternion_matrix(
    #                 [rot[0], rot[1], rot[2], rot[3]]
    #             )
    #             rot_matrix[0, 3] = trans[0]
    #             rot_matrix[1, 3] = trans[1]
    #             rot_matrix[2, 3] = trans[2]
    #             target_T_source = rot_matrix.copy()
    #             success = True
    #         except (
    #             tf.LookupException,
    #             tf.ConnectivityException,
    #             tf.ExtrapolationException,
    #         ):
    #             rospy.logwarn(
    #                 rospy.get_name()
    #                 + " lookupTransform from %s to %s failed"
    #                 % (target_frame, source_frame)
    #             )
    #         time.sleep(sleep_time)
    #     return target_T_source

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

        self.publish_hitbot_raw(
            self.scara_arm.encoder_x,
            self.scara_arm.encoder_y,
            self.scara_arm.encoder_z,
            self.scara_arm.encoder_r,
        )

    def publish_hitbot_raw(self, x, y, z, r):
        hitbot_raw_pose_msg = HitbotPoseRaw()
        hitbot_raw_pose_msg.x = x
        hitbot_raw_pose_msg.y = y
        hitbot_raw_pose_msg.z = z
        hitbot_raw_pose_msg.r = r
        self.hitbot_raw_poses_pub.publish(hitbot_raw_pose_msg)

    def publish_joint_states(self):
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

        self.joint_states_pub.publish(joint_state)

    def scara_control_execute(self, scara_command: HitbotCommand):
        x = scara_command.goal_x  # mm
        y = scara_command.goal_y  # mm
        z = scara_command.goal_z  # mm
        r = scara_command.goal_r  # mm
        speed = scara_command.moving_speed  # mm/s or deg/s
        target_speed_ratio = scara_command.target_speed_ratio  # 0-1
        hand_type = scara_command.hand_type

        postion1 = scara_command.claw_postion
        velocity1 = scara_command.claw_velocity
        current1 = scara_command.claw_current
        # ret = self.scara_arm.new_movej_xyz_lr(
        #     x, y, z, r, speed, target_speed_ratio, hand_type
        # )
        ret = self.scara_arm.movel_xyz(x, y, z, r, speed)
        self.scara_arm.wait_stop()
        self.claw_control(postion1, velocity1, current1)

        self.return_logging(scara_command, ret)

        # self.scara_arm.wait_stop()

    def move_to_target_service(self, target_request):
        response = MoveToTargetPoseServiceResponse()

        # hitbot command
        x = target_request.hitbot_command.goal_x  # mm
        y = target_request.hitbot_command.goal_y  # mm
        z = target_request.hitbot_command.goal_z  # mm
        r = target_request.hitbot_command.goal_r  # mm
        speed = target_request.hitbot_command.moving_speed  # mm/s or deg/s
        target_speed_ratio = target_request.hitbot_command.target_speed_ratio  # 0-1
        hand_type = target_request.hitbot_command.hand_type

        postion1 = target_request.hitbot_command.claw_postion
        velocity1 = target_request.hitbot_command.claw_velocity
        current1 = target_request.hitbot_command.claw_current
        return_status = self.point_rotate_test(x, y, r)
        # print(return_status)
        if return_status != 0:
            ret = self.scara_arm.new_movej_xyz_lr(
                x, y, z, r, speed, target_speed_ratio, hand_type
            )
            self.scara_arm.wait_stop()
            # start_time = time.perf_counter()
            self.claw_control(postion1, velocity1, current1)
            self.return_logging(target_request.hitbot_command, ret)
            print("ret:", ret)
            if ret != target_request.hitbot_command.VALID:
                msg_replied = " The command was NOT executed successfully!"
                rospy.logwarn(rospy.get_name() + msg_replied)
                response.success = False
                response.message = msg_replied
            else:
                response.success = True
                response.message = "The command was executed successfully!"

        return response

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

    def point_rotate_test(self, goal_x, goal_y, goal_angle_to_base):
        # !collision_test coordinate is different to urdf coordinate
        # !But Their coordinate system origin is the same, all on base link
        # !urdf corrdinate     | X
        # !                    |
        # !             Y______|

        # !collision_test corrdinate     | Y
        # !                              |
        # !                              |______X

        # current position and angle
        current_x = self.x_to_base
        current_y = self.y_to_base
        current_angle_to_base = self.angle_ee_to_base
        print("initial position:", -current_y, current_x)
        ee_origin = Point(-current_y, current_x)
        # pillar's position relative to link1
        # refer to https://www.hitbot.cc/assets/download/z-arm/z-arm-2442/2442.pdf
        body_box = box(-60.0, -180.0, 60.0, -60.0)

        ee_box_origin = box(
            ee_origin.x - 30,
            ee_origin.y - 50,
            ee_origin.x + 30,
            ee_origin.y + 170,
        )
        # current_x, current_y = ee_box_origin.boundary.xy

        # rotating angle to link1 is equal to baselink
        ee_box_to_link1 = affinity.rotate(
            ee_box_origin, current_angle_to_base + self.EE_OFFSET, ee_origin
        )
        # translate the ee box
        target_point = Point(-goal_y, goal_x)
        ee_box_to_link1 = affinity.translate(
            ee_box_to_link1,
            target_point.x - ee_origin.x,
            target_point.y - ee_origin.y,
        )
        # x2, y2 = transform_a.boundary.xy
        # x3, y3 = body_box.boundary.xy
        ee_box_to_link1_check = affinity.rotate(
            ee_box_to_link1,
            goal_angle_to_base - current_angle_to_base,
            target_point,
            False,
        )
        # x6, y6 = ee_box_to_link1_check.boundary.xy
        collision_check = ee_box_to_link1_check.intersects(body_box)
        print("collision_check", collision_check)
        if collision_check == True:
            # collision_part = ee_box_to_link1_check.boundary.intersection(
            #     body_box.boundary
            # )
            # print(collision_part)
            return_state = 0
            rospy.logwarn(
                rospy.get_name() + "The goal position is in collision with body"
            )
        else:
            limit_angle_LH = 0
            limit_angle_RH = 0
            angle_check_delta = 1.5
            while ee_box_to_link1_check.intersects(body_box) == False:
                ee_box_to_link1_rotate_check = affinity.rotate(
                    ee_box_to_link1_check, limit_angle_LH, target_point, False
                )
                limit_angle_LH = limit_angle_LH + angle_check_delta
                # x4, y4 = ee_box_to_link1_rotate_check.boundary.xy
                if ee_box_to_link1_rotate_check.intersects(body_box) == True:
                    limit_angle_LH = limit_angle_LH + goal_angle_to_base
                    print("limit_angle_LH:", limit_angle_LH)
                    return_state = 2
                    break
                if limit_angle_LH > 360:
                    return_state = 1
                    rospy.loginfo(rospy.get_name() + " The robotic arm is rotatable")
                    break
            while ee_box_to_link1_check.intersects(body_box) == False:
                ee_box_to_link1_rotate_check = affinity.rotate(
                    ee_box_to_link1_check, limit_angle_RH, target_point, False
                )
                limit_angle_RH = limit_angle_RH - angle_check_delta
                # x5, y5 = ee_box_to_link1_rotate_check.boundary.xy
                if ee_box_to_link1_rotate_check.intersects(body_box) == True:
                    limit_angle_RH = limit_angle_RH + goal_angle_to_base
                    print("limit_angle_RH:", limit_angle_RH)
                    return_state = 1
                    break
                if limit_angle_RH < -360:
                    return_state = 2
                    break
            if return_state == 1:
                rospy.logwarn(
                    rospy.get_name()
                    + " The robotic arm is not rotatable"
                    + " Maximum angle: "
                    + str(limit_angle_LH)
                    + " Minimum Angle: "
                    + str(limit_angle_RH)
                )
        return return_state

    def claw_control(self, postion, velocity, current):
        # ee2 = b"\x00\x00\x00\x00\x00\x00\x00\x00"
        if postion >= 25:
            self.claw_velocity_control(velocity)
            # ret = self.scara_arm.com485_recv(ee2)
            # print("receive=", ret, ee2)
            self.claw_current_control(current)

        self.claw_postion_control(postion)

    # s
    def claw_postion_control(self, set_postion):
        move_dis_cmd = b"\x01\x10\x00\x02\x00\x02\x04"
        tr = struct.pack(">f", set_postion)
        move_dis_cmd += tr

        command_crc0 = libscrc.modbus(move_dis_cmd).to_bytes(2, "little")
        move_dis_cmd += command_crc0
        # start_time = time.perf_counter()
        self.scara_arm.com485_send(move_dis_cmd, 13)
        # stop_time = time.perf_counter()
        # run_time = stop_time - start_time
        # print("running_time:  ", run_time)

    def claw_velocity_control(self, set_velocity):
        move_vel_cmd = b"\x01\x10\x00\x04\x00\x02\x04"
        tr2 = struct.pack(">f", set_velocity)
        move_vel_cmd += tr2
        command_crc1 = libscrc.modbus(move_vel_cmd).to_bytes(2, "little")
        move_vel_cmd += command_crc1
        self.scara_arm.com485_send(move_vel_cmd, 13)

    def claw_current_control(self, set_current):
        move_cur_cmd = b"\x01\x10\x00\x06\x00\x02\x04"
        tr3 = struct.pack(">f", set_current)
        move_cur_cmd += tr3
        command_crc2 = libscrc.modbus(move_cur_cmd).to_bytes(2, "little")
        move_cur_cmd += command_crc2
        # print(move_cur_cmd)
        self.scara_arm.com485_send(move_cur_cmd, 13)
        # set_last_current = set_current


if __name__ == "__main__":
    abs_lib_path = (
        roslib.packages.get_pkg_dir("hitbot_scara_control") + "/script/hitbot_interface"
    )
    sys.path.append(abs_lib_path)
    # product id
    robot_id = 190
    hitbot = ScaraController(robot_id, abs_lib_path)
    # hitbot.scara_arm.joint_home(1)
    # hitbot.scara_arm.joint_home(2)
    # hitbot.scara_arm.joint_home(3)
    # hitbot.scara_arm.joint_home(4)
    # hitbot.scara_arm.set_drag_teach(True)
    # print(hitbot.scara_arm.encoder_r)
    while not rospy.is_shutdown():
        hitbot.publish_joint_states()
        # print(
        #     hitbot.lookup_transformation("/camera_link", "/base_link", sleep_time=0.5)
        # )
        hitbot.control_rate.sleep()
