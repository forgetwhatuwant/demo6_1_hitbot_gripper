#!/usr/bin/env python
import rospy
import copy
from sensor_msgs.msg import JointState

class JointCommandUpsampler:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("joint_command_upsampler")

        # Subscribe to the low-rate joint command topic
        self.low_rate_sub = rospy.Subscriber("/hitbot_scara/joint_command_low_rate", JointState, self.low_rate_callback)

        # Publish the upsampled joint command topic
        self.high_rate_pub = rospy.Publisher("/hitbot_scara/joints_command", JointState, queue_size=1)

        # Set low and high rates (assuming fixed values)
        self.low_rate = rospy.get_param("~low_rate", 30)
        self.high_rate = rospy.get_param("~high_rate", 500)

        # Calculate time step based on the ratio of low to high rate
        self.incremental = self.low_rate / self.high_rate
        self.steps = 0

        # Initialize variables
        self.last_joint_command = None
        self.previous_joint_command = None

        # Create ROS rate object for loop control (set to high rate)
        self.loop_rate = rospy.Rate(self.high_rate)

    def low_rate_callback(self, joint_state):
        self.previous_joint_command = copy.copy(self.last_joint_command)
        self.last_joint_command = joint_state
        self.steps = 0
        # if self.last_joint_command is not None and self.previous_joint_command is not None:

        #     print("-----------------------")
        #     print("previous_joint_command: ", self.previous_joint_command.position[0])
        #     print("last_joint_command: ", self.last_joint_command.position[0])

    def main_loop(self):
        while not rospy.is_shutdown():
            # Call low_rate_callback to process incoming messages (optional)
            #rospy.spin_once()  # This processes a single callback from the queue

            # Process data and publish upsampled message
            self.upsample_and_publish()

            # Sleep to maintain desired loop rate (high rate)
            self.loop_rate.sleep()

    def upsample_and_publish(self):
        if self.last_joint_command is not None and self.previous_joint_command is not None:
            # Perform linear interpolation based on pre-calculated time step
            num_joints = len(self.last_joint_command.position)
            high_rate_msg = JointState()
            high_rate_msg.header = self.last_joint_command.header
            high_rate_msg.name = self.last_joint_command.name
            high_rate_msg.position = []
            for i in range(num_joints):
                previous_pos = self.previous_joint_command.position[i]
                current_pos = self.last_joint_command.position[i]
                # Interpolate based on time step
                upsampled_pos = previous_pos + (current_pos - previous_pos) * self.incremental * self.steps
                self.steps = min(self.high_rate/self.low_rate, self.steps)
                high_rate_msg.position.append(upsampled_pos)

            self.steps += 1

            # Publish the upsampled message
            self.high_rate_pub.publish(high_rate_msg)

if __name__ == "__main__":
    try:
        upsampler = JointCommandUpsampler()
        upsampler.main_loop()
    except rospy.ROSInterruptException:
        pass