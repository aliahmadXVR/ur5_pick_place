#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion
import subprocess
import copy

rospy.init_node('line_move', anonymous=True)
print ('Init Done')

robot = moveit_commander.RobotCommander()
group_name = "ur5_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
gripper_group = moveit_commander.MoveGroupCommander('gripper')

home_joint_angles = [-1.50, -1.50, 1.50, -1.55, -1.55, 0.0] 
pick_joint_angles = [-1.5024685883829463, -1.288272797025268, 1.972095616995679, -2.217765520408774, -1.5496723128543648, -0.0001668693358825024]
place_joint_angles = [-1.91342252110476, -1.3238122461523005, 2.0161890844310726, -2.2261916459086066, -1.5627755109263948, -0.4103684155957481]
#First Execute the Homing
move_group.set_joint_value_target(home_joint_angles)
plan = move_group.plan()
move_group.execute(plan)
rospy.sleep(2)
move_group.stop()

#Open the Gripper
gripper_joint_goal = [0.02] #Open Joint Position
gripper_group.set_joint_value_target(gripper_joint_goal)
gripper_group.go()

# Execute the Pick Pose
move_group.set_joint_value_target(pick_joint_angles)
plan = move_group.plan()
move_group.execute(plan)
rospy.sleep(2)
move_group.stop()

#Close the Gripper
gripper_joint_goal = [0.35] #Open Joint Position 0.45
gripper_group.set_joint_value_target(gripper_joint_goal)
gripper_group.go()


waypoints = []
scale = 1

wpose = move_group.get_current_pose().pose
# wpose.position.z -= scale * 0.1  # First move up (z)
# # wpose.position.y += scale * 0.2  # and sideways (y)
# waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * -0.5  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

# wpose.position.y -= scale * 0.1  # Third move sideways (y)
# waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
(plan, fraction) = move_group.compute_cartesian_path(
    waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
)  # jump_threshold

move_group.execute(plan, wait=True)

moveit_commander.roscpp_shutdown()


