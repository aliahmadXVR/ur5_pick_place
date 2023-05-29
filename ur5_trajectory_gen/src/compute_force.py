#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
import select
import termios
import tty
from geometry_msgs.msg import WrenchStamped

desired_force_limit = 5.0  # Desired force limit in Newtons

def wrench_callback(wrench_msg):
    force_magnitude = (wrench_msg.wrench.force.x ** 2 +
                       wrench_msg.wrench.force.y ** 2 +
                       wrench_msg.wrench.force.z ** 2) ** 0.5

    print('Force Mag= ',force_magnitude)

    # if force_magnitude > desired_force_limit:
    #     # Apply corrective action here (e.g., slow down or stop the gripper motion)
    #     # Modify the gripper's velocity or acceleration to reduce the force

    #     # Example: Stop the gripper
    #     stop_gripper()

def stop_gripper():
    # Code to stop the gripper motion
    pass

def main():
    rospy.init_node('force_control_node')
    rospy.Subscriber('/ft_sensor/raw', WrenchStamped, wrench_callback)
    rospy.spin()

if __name__ == '__main__':
    main()