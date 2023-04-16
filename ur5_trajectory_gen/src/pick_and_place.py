#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def pick_and_place():
    # Initialize ROS node
    rospy.init_node('pick_and_place', anonymous=True)

    # Create a MoveGroupCommander for the UR5 robot
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    # Set the planning time and number of planning attempts
    move_group.set_planning_time(5.0)
    move_group.set_num_planning_attempts(10)

    # Define the pick and place target positions
    pick_pose = geometry_msgs.msg.PoseStamped()
    pick_pose.header.frame_id = "world"
    pick_pose.pose.position.x = 0.5
    pick_pose.pose.position.y = -0.3
    pick_pose.pose.position.z = 0.5
    pick_pose.pose.orientation.x = 0.0
    pick_pose.pose.orientation.y = 0.707
    pick_pose.pose.orientation.z = 0.0
    pick_pose.pose.orientation.w = 0.707

    place_pose = geometry_msgs.msg.PoseStamped()
    place_pose.header.frame_id = "world"
    place_pose.pose.position.x = 0.5
    place_pose.pose.position.y = 0.3
    place_pose.pose.position.z = 0.5
    place_pose.pose.orientation.x = 0.0
    place_pose.pose.orientation.y = 0.0
    place_pose.pose.orientation.z = 0.0
    place_pose.pose.orientation.w = 1.0

    # Set the pick and place target positions as the target pose
    move_group.set_pose_target(pick_pose, "gripper_link")

    # Plan and execute the pick trajectory
    pick_plan = move_group.go(wait=True)
    if pick_plan:
        rospy.loginfo("Pick trajectory planning and execution succeeded")
    else:
        rospy.logerr("Pick trajectory planning and execution failed")
        return

    # Set the place target position as the target pose
    move_group.set_pose_target(place_pose, "gripper_link")

    # Plan and execute the place trajectory
    place_plan = move_group.go(wait=True)
    if place_plan:
        rospy.loginfo("Place trajectory planning and execution succeeded")
    else:
        rospy.logerr("Place trajectory planning and execution failed")
        return

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
