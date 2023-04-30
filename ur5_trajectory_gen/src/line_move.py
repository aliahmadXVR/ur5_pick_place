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
pick_joint_angles = [-1.1520582570070497, -1.3718278606223802, 2.0967660701658684, -2.2530223383587824, -1.5425112660928555, 0.3649237989447984]
place_joint_angles = [-1.8373808770962263, -1.2333595828936792, 1.9591080802974492, -2.24350744482176, -1.5545188647636587, -0.2821204263605024]

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
wpose.position.z -= scale * 0.1  # First move up (z)
# wpose.position.y += scale * 0.2  # and sideways (y)
waypoints.append(copy.deepcopy(wpose))

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


