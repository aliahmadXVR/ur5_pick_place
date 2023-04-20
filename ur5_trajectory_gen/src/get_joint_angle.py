#!/usr/bin/env python

import rospy
import moveit_commander

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('get_joint_angles', anonymous=True)

    # Create a MoveGroupCommander for the UR5 robot
    move_group = moveit_commander.MoveGroupCommander("ur5_arm")

    # Get the current joint values
    joint_values = move_group.get_current_joint_values()

    # Print the joint values
    print("Current joint values:")
    print(joint_values)

    # Shutdown MoveIt!
    moveit_commander.roscpp_shutdown()