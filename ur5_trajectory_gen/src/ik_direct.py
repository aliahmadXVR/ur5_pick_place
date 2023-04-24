#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

if __name__ == '__main__':
    try:
        # Initialize MoveIt!
        moveit_commander.roscpp_initialize(sys.argv)

        # Create a MoveGroupCommander for the UR5 robot
        move_group = moveit_commander.MoveGroupCommander("ur5_arm")

        # Set the planning time and number of planning attempts
        move_group.set_planning_time(5.0)
        move_group.set_num_planning_attempts(10)

        # Set the reference frame and end effector link
        move_group.set_pose_reference_frame("world")
        move_group.set_end_effector_link("ee_link")

        # Define the target pose
        pick_pose.position.x = 0.147
        pick_pose.position.y = -0.506
        pick_pose.position.z = 0.428
        pick_pose.orientation.x = 0.492
        pick_pose.orientation.y = 0.517
        pick_pose.orientation.z = -0.471
        pick_pose.orientation.w = 0.516

        # Set the target pose as the target
        #move_group.set_pose_target(pick_pose)

        # Compute the IK for the target pose
        plan = move_group.go(wait=True)

        print (plan)
        joint_goal = move_group.get_current_joint_values()
        print("joint gaol = ")
        print (joint_goal[5])
        # If the plan was successful, execute the trajectory
        # if plan:
        #     rospy.loginfo("Trajectory planning succeeded")

        #     # Set the joint goal to move the gripper to a specific position
        #     joint_goal = move_group.get_current_joint_values()
        #     joint_goal[5] = 0.5 # set the desired position of gripper_joint
        #     move_group.go(joint_goal, wait=True)

        #     rospy.loginfo("Gripper moved to the desired position")

        # else:
        #     rospy.logerr("Trajectory planning failed")
        
        # Shutdown MoveIt!
        moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        pass
