#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion
import subprocess

rospy.init_node('ur5_moveit_demo', anonymous=True)
print ('Init Done')

robot = moveit_commander.RobotCommander()
group_name = "ur5_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
gripper_group = moveit_commander.MoveGroupCommander('gripper')

home_joint_angles = [-1.50, -1.50, 1.50, -1.55, -1.55, 0.0] 
pick_joint_angles = [-1.5049656751450744, -1.2540979113282544, 1.997585759467615, -2.2709904328346155, -1.5509109895384547, 0.003967209252296655]
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

gripping_node = subprocess.Popen(['rosrun', 'ur5_trajectory_gen', 'mimic_grip.py'])
rospy.sleep(2)

#Got to Home Position again
move_group.set_joint_value_target(home_joint_angles)
plan = move_group.plan()
move_group.execute(plan)
rospy.sleep(2)
move_group.stop()

# Move to Place Position
move_group.set_joint_value_target(place_joint_angles)
plan = move_group.plan()
move_group.execute(plan)
rospy.sleep(2)
move_group.stop()

#Open the Gripper
gripper_joint_goal = [0.02] #Open Joint Position
gripper_group.set_joint_value_target(gripper_joint_goal)
gripper_group.go()

#Terminate the gripping
gripping_node.terminate()


#Got to Home Position again
move_group.set_joint_value_target(home_joint_angles)
plan = move_group.plan()
move_group.execute(plan)
rospy.sleep(2)
move_group.stop()

moveit_commander.roscpp_shutdown()


