#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64

position_1 = [-0.2, -1, -1, -1]
position_2 = [-0.05, 1, 1, 1]
# position_2 = [-0.1,1,-1,1]


def position_publisher():
    rospy.init_node("position_publisher", anonymous=True)

    while not rospy.is_shutdown():
        publish_joint_position("joint1", position_1[0])
        publish_joint_position("joint2", position_1[1])
        publish_joint_position("joint3", position_1[2])
        publish_joint_position("joint4", position_1[3])
        rospy.sleep(5)

        publish_joint_position("joint1", position_2[0])
        publish_joint_position("joint2", position_2[1])
        publish_joint_position("joint3", position_2[2])
        publish_joint_position("joint4", position_2[3])
        rospy.sleep(5)


def publish_joint_position(joint_name, position):
    pub = rospy.Publisher(
        "/" + joint_name + "_position_controller/command", Float64, queue_size=10
    )
    message = Float64()
    message.data = position
    pub.publish(message)


if __name__ == "__main__":
    try:
        position_publisher()
    except rospy.ROSInterruptException:
        pass
