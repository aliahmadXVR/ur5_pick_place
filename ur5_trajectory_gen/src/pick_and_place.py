#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion


rospy.init_node('ur5_moveit_demo', anonymous=True)
print ('Init Done')

robot = moveit_commander.RobotCommander()
group_name = "ur5_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
gripper_group = moveit_commander.MoveGroupCommander('gripper')

home_joint_angles = [-1.50, -1.50, 1.50, -1.55, -1.55, 0.0] 
# pick_joint_angles = [-1.5026816953804882, -1.2896108217907631, 1.97276934331137, -2.221730020476911, -1.549604572181308, 0.0007375337770314516]
pick_joint_angles = [-1.5049656751450744, -1.2540979113282544, 1.997585759467615, -2.2709904328346155, -1.5509109895384547, 0.003967209252296655]
place_joint_angles = [-1.50, -1.50, 1.50, -1.55, -1.55, 0.0]

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

# #Close the Gripper
# gripper_joint_goal = [0.45] #Open Joint Position
# gripper_group.set_joint_value_target(gripper_joint_goal)
# gripper_group.go()

# #Got to Home Position again
# move_group.set_joint_value_target(home_joint_angles)
# plan = move_group.plan()
# move_group.execute(plan)
# rospy.sleep(2)
# move_group.stop()


moveit_commander.roscpp_shutdown()




# Pick Joint angles
# [-1.5026816953804882, -1.2896108217907631, 1.97276934331137, -2.221730020476911, -1.549604572181308, 0.0007375337770314516]

###--------------------In Case Joint Constraints are required-----------------####
 # Define joint limits
    # joint_limits = moveit_msgs.msg.Constraints()
    # joint_limits.name = "joint_limits"
    # joint_limits.joint_constraints.append(
    #     moveit_msgs.msg.JointConstraint(
    #         joint_name="wrist_2_joint",
    #         position=-1.57,
    #         tolerance_above=-1.74,
    #         tolerance_below=-1.39,
    #         weight=1.0           
    #     )
    # )
