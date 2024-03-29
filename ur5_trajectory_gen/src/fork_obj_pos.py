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

# home_joint_angles = [ -1.5011178815242383, -1.3739078517056527, 1.8975074771314553, -2.058466662963273, -1.5499322153193642, -0.0014575012444089097]
home_joint_angles = [-1.5028521223100038, -1.3606884375055799, 1.9167339325438757, -2.084721616001108, -1.5497631380021826, 8.967910447932326e-05]
place_joint_angles = [-1.8373808770962263, -1.2333595828936792, 1.9591080802974492, -2.24350744482176, -1.5545188647636587, -0.2821204263605024]


init_pose = geometry_msgs.msg.Pose()
init_pose.position.x = 0.030 
init_pose.position.y = -0.670 
init_pose.position.z = 0.800 
init_pose.orientation.w = 1.0

start_pose = Pose()
start_pose.position.x = 0.817
start_pose.position.y = 0.191
start_pose.position.z = 0.43
start_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

end_pose = Pose()
end_pose.position.x = 0.4
end_pose.position.y = 0.2
end_pose.position.z = 0.43
end_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

waypoints = [start_pose, end_pose]

# move_group.set_pose_target(init_pose)

move_group.set_joint_value_target(home_joint_angles)

# Plan and execute the trajectory
plan = move_group.go(wait=True)
if plan:
    print("Successfully planned and executed trajectory!")
else:
    print("Failed to plan and execute trajectory.")


rospy.sleep(2)
# Stop the robot
move_group.stop()

print("executing next command")

#Close the Gripper
gripper_joint_goal = [0.71] #Open Joint Position
gripper_group.set_joint_value_target(gripper_joint_goal)
gripper_group.go()
move_group.stop()

gripping_node = subprocess.Popen(['rosrun', 'ur5_trajectory_gen', 'mimic_grip_fork.py'])
rospy.sleep(2)
move_group.stop()

# # Execute the Pick Pose
# move_group.set_joint_value_target(place_joint_angles)
# plan = move_group.plan()
# move_group.execute(plan)
# rospy.sleep(2)
# move_group.stop()

# Clean up MoveIt Commander
move_group.clear_pose_targets()
rospy.sleep(1)
moveit_commander.roscpp_shutdown()











########---------------------------------------------------##
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# from geometry_msgs.msg import Pose, Point, Quaternion


# rospy.init_node('ur5_moveit_demo', anonymous=True)
# print ('Init Done')

# # Initialize MoveIt Commander
# robot = moveit_commander.RobotCommander()
# group_name = "ur5_arm"
# move_group = moveit_commander.MoveGroupCommander(group_name)

# # Set target joint angles
# home_joint_angles = [0.0, -1.54, 1.54, -1.57, -1.57, 0.0]  # Specify desired joint angles here

# # Set a Initial pose for the end-effector
# init_pose = geometry_msgs.msg.Pose()
# init_pose.position.x = 0.49   #0.599
# init_pose.position.y = 0.10  #-0.100
# init_pose.position.z = 0.43  #0.0250
# init_pose.orientation.w = 1.0

# start_pose = Pose()
# start_pose.position.x = 1.0
# start_pose.position.y = 0.0
# start_pose.position.z = 0.43
# start_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

# end_pose = Pose()
# end_pose.position.x = 1.0
# end_pose.position.y = 1.0
# end_pose.position.z = 0.43
# end_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

# # Define the waypoints as a list of poses
# waypoints = [start_pose, end_pose]

# # Set the step size for the Cartesian path
# step_size = 0.1

# # Compute the Cartesian path
# path = move_group.compute_cartesian_path(waypoints, step_size, 0.0)

# # #Move to Init pose
# # move_group.set_pose_target(init_pose)

# # # Set Homing joint angles
# # move_group.set_joint_value_target(home_joint_angles)

# # # Plan and execute the trajectory
# # plan = move_group.go(wait=True)
# # if plan:
# #     print("Successfully planned and executed trajectory!")
# # else:
# #     print("Failed to plan and execute trajectory.")

# # Execute the computed path
# move_group.execute(path)

# # Stop the robot
# move_group.stop()


# # Clean up MoveIt Commander
# move_group.clear_pose_targets()
# rospy.sleep(1)
# moveit_commander.roscpp_shutdown()


# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# from geometry_msgs.msg import Pose, Point, Quaternion


# rospy.init_node('ur5_moveit_demo', anonymous=True)
# print ('Init Done')

# # Initialize MoveIt Commander
# robot = moveit_commander.RobotCommander()
# group_name = "ur5_arm"
# move_group = moveit_commander.MoveGroupCommander(group_name)

# # move_group.set_goal_tolerance(0.04)

# # Set target joint angles
# home_joint_angles = [-1.50, -1.50, 1.50, -1.55, -1.55, 0.0]  # Specify desired joint angles here

# # Set a Initial pose for the end-effector
# init_pose = geometry_msgs.msg.Pose()
# init_pose.position.x = 0.817#0.49   #0.599
# init_pose.position.y = 0.191##0.10  #-0.100
# init_pose.position.z = -0.006#0.43  #0.0250
# init_pose.orientation.w = 1.0

# start_pose = Pose()
# start_pose.position.x = 0.817
# start_pose.position.y = 0.191
# start_pose.position.z = 0.43
# start_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

# end_pose = Pose()
# end_pose.position.x = 0.4
# end_pose.position.y = 0.2
# end_pose.position.z = 0.43
# end_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

# # Define the waypoints as a list of poses
# waypoints = [start_pose, end_pose]

# #Move to Init pose
# # move_group.set_pose_target(init_pose)

# # Set Homing joint angles
# # move_group.set_joint_value_target(home_joint_angles)

# # Plan and execute the trajectory
# plan = move_group.go(wait=True)
# if plan:
#     print("Successfully planned and executed trajectory!")
# else:
#     print("Failed to plan and execute trajectory.")


# rospy.sleep(2)
# # Stop the robot
# move_group.stop()

# print("executing next command")

# # print(waypoints)
# # Set the step size for the Cartesian path
# step_size = 0.01

# # Compute the Cartesian path
# (path,fraction) = move_group.compute_cartesian_path(waypoints, step_size, 0.0)

# # Execute the computed path
# move_group.execute(path, wait=True)

# # Stop the robot
# move_group.stop()


# # Clean up MoveIt Commander
# move_group.clear_pose_targets()
# rospy.sleep(1)
# moveit_commander.roscpp_shutdown()
