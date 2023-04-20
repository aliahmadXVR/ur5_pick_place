#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def pick_and_place():
    # Initialize ROS node
    rospy.init_node('pick_and_place', anonymous=True)

    # Create a MoveGroupCommander for the UR5 robot
    move_group = moveit_commander.MoveGroupCommander("ur5_arm")

    # Set the planning time and number of planning attempts
    move_group.set_planning_time(5.0)
    move_group.set_num_planning_attempts(10)

    # Set the reference frame and end effector link
    move_group.set_pose_reference_frame("world")
    move_group.set_end_effector_link("ee_link")

    # Define the pick and place target positions
    pick_pose = geometry_msgs.msg.PoseStamped()
    pick_pose.header.frame_id = "world"
    pick_pose.pose.position.x = 0.11
    pick_pose.pose.position.y = -0.71
    pick_pose.pose.position.z = 0.18
    pick_pose.pose.orientation.x = 0.0
    pick_pose.pose.orientation.y = 0.0
    pick_pose.pose.orientation.z = 0.0
    pick_pose.pose.orientation.w = 1.0

    place_pose = geometry_msgs.msg.PoseStamped()
    place_pose.header.frame_id = "world"
    place_pose.pose.position.x = 0.11
    place_pose.pose.position.y = -0.71
    place_pose.pose.position.z = 0.18
    place_pose.pose.orientation.x = 0.0
    place_pose.pose.orientation.y = 0.0
    place_pose.pose.orientation.z = 0.0
    place_pose.pose.orientation.w = 1.0

    # Set the pick and place target positions as the target pose
    move_group.set_pose_target(pick_pose, "ee_link")

    # Define joint limits
    joint_limits = moveit_msgs.msg.Constraints()
    joint_limits.name = "joint_limits"
    joint_limits.joint_constraints.append(
        moveit_msgs.msg.JointConstraint(
            joint_name="wrist_2_joint",
            position=0.0,
            tolerance_above=-1.74533,
            tolerance_below=1.39626,
            weight=1.0           
        )
    )

    joint_limits = moveit_msgs.msg.Constraints()
    joint_limits.name = "joint_limits"
    joint_limits.joint_constraints.append(
        moveit_msgs.msg.JointConstraint(
            joint_name="wrist_1_joint",
            position=0.0,
            tolerance_above=-1.74533,
            tolerance_below=1.39626,
            weight=1.0
        )
    )

    joint_limits = moveit_msgs.msg.Constraints()
    joint_limits.name = "joint_limits"
    joint_limits.joint_constraints.append(
        moveit_msgs.msg.JointConstraint(
            joint_name="shoulder_lift_joint",
            position=0.0,
            tolerance_above=-0.87,
            tolerance_below=-1.57,
            weight=1.0
        )
    )
    # Apply joint limits to the robot
    move_group.set_path_constraints(joint_limits)




    # Plan and execute the pick trajectory
    pick_plan = move_group.go(wait=True)
    if pick_plan:
        rospy.loginfo("Pick trajectory planning and execution succeeded")
    else:
        rospy.logerr("Pick trajectory planning and execution failed")
        return

    # Set the place target position as the target pose
    move_group.set_pose_target(place_pose, "ee_link")

    # # Plan and execute the place trajectory
    # place_plan = move_group.go(wait=True)
    # if place_plan:
    #     rospy.loginfo("Place trajectory planning and execution succeeded")
    # else:
    #     rospy.logerr("Place trajectory planning and execution failed")
    #     return

    rospy.loginfo("Pick and place operation completed successfully")

if __name__ == '__main__':
    try:
        # Initialize MoveIt!
        moveit_commander.roscpp_initialize(sys.argv)

        # Call the pick_and_place function
        pick_and_place()

        # Shutdown MoveIt!
        moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        pass
