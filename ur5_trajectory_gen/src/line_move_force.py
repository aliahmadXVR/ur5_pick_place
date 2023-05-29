#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
from geometry_msgs.msg import Pose, Point, Quaternion, WrenchStamped
import subprocess
import copy
from std_msgs.msg import Float64
import time


desired_force = -11.0  # Desired constant force in Newtons -11 had good result

class ForceControlNode:
    def __init__(self):
        rospy.init_node('force_control_node')
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("ur5_arm")  # Replace "manipulator" with the name of your UR5 planning group
        self.sub_wrench = rospy.Subscriber('/ft_sensor/raw', WrenchStamped, self.wrench_force_callback)
        self.error_pub =  rospy.Publisher('/force_error', Float64 , queue_size=10)
        self.force_error = 0.0
        self.home_joint_angles = [-1.50, -1.50, 1.50, -1.55, -1.55, 0.0] 
        self.pick_joint_angles = [-1.4997324032807544, -1.2680472001527, 1.9598007170315341, -2.2199747168046464, -1.5555591206423127, 0.00322782140316491]
        # self.pick_joint_angles = [-1.4999106396561297, -1.2255297314169198, 2.0082943183565654, -2.32396641177049, -1.549817489440592, 0.0007815502676509212] #into table
    def wrench_force_callback(self, wrench_msg):
        force_z = wrench_msg.wrench.force.y   #we have force in y
        self.force_error = force_z  -desired_force
        self.error_pub.publish(self.force_error)

    
    def goto_home_pose(self):
        self.group.set_joint_value_target(self.home_joint_angles)
        plan = self.group.plan()
        self.group.execute(plan)
        rospy.sleep(0.5)
        self.group.stop()
    
    def goto_pick_pose(self):
        self.group.set_joint_value_target(self.pick_joint_angles)
        plan = self.group.plan()
        self.group.execute(plan)
        rospy.sleep(0.5)
        self.group.stop()

    def raster_line(self):
        waypoints = []
        scale = 1
        wpose = self.group.get_current_pose().pose
        wpose.position.x += scale * -0.5  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        # Move the end effector along the desired path while maintaining a constant force
        while not rospy.is_shutdown():
            (plan, fraction) = self.group.compute_cartesian_path(
                waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
            )  # jump_threshold

            self.group.execute(plan, wait=False)

            # if abs(self.force_error) <= threshold:
            #     break

            # Adjust the target pose based on the force error
            delta_z = self.force_error * -0.008  # Example adjustment based on the force error 0.01
            wpose.position.z += delta_z
            waypoints[0] = copy.deepcopy(wpose)

            # Add a small delay to allow the robot to respond to the previous adjustment
            time.sleep(0.1)

def main():
    node = ForceControlNode()
    rospy.sleep(0.5)  # Allow time for publishers and subscribers to connect

    node.goto_home_pose() 

    node.goto_pick_pose()

    node.raster_line()



    # # Specify the target pose of the end effector in a straight line
    # target_pose = geometry_msgs.msg.Pose()
    # target_pose.position.x = 0.5  # Modify the target position as desired
    # target_pose.position.y = 0.0
    # target_pose.position.z = 0.1

    # node.move_end_effector(target_pose)

    

if __name__ == '__main__':
    main()


##################################################################################3

# rospy.init_node('line_move_force', anonymous=True)
# print ('Init Done')

# robot = moveit_commander.RobotCommander()
# group_name = "ur5_arm"
# move_group = moveit_commander.MoveGroupCommander(group_name)
# gripper_group = moveit_commander.MoveGroupCommander('gripper')

# home_joint_angles = [-1.50, -1.50, 1.50, -1.55, -1.55, 0.0] 
# pick_joint_angles = [-1.4182433487645936, -1.414314673957275, 2.1978555438286094, -2.3164678808406984, -1.5489462387244384, 0.10146907540718964]
# place_joint_angles = [-1.8373808770962263, -1.2333595828936792, 1.9591080802974492, -2.24350744482176, -1.5545188647636587, -0.2821204263605024]

# #First Execute the Homing
# move_group.set_joint_value_target(home_joint_angles)
# plan = move_group.plan()
# move_group.execute(plan)
# rospy.sleep(0.5)
# move_group.stop()

# #Open the Gripper
# gripper_joint_goal = [0.02] #Open Joint Position
# gripper_group.set_joint_value_target(gripper_joint_goal)
# gripper_group.go()

# # Execute the Pick Pose
# move_group.set_joint_value_target(pick_joint_angles)
# plan = move_group.plan()
# move_group.execute(plan)
# rospy.sleep(0.5)
# move_group.stop()

# #Close the Gripper
# gripper_joint_goal = [0.60] #Open Joint Position 0.45
# gripper_group.set_joint_value_target(gripper_joint_goal)
# gripper_group.go()


# waypoints = []
# scale = 1

# wpose = move_group.get_current_pose().pose
# # wpose.position.z -= scale * 0.1  # First move up (z)
# # wpose.position.y += scale * 0.2  # and sideways (y)
# # waypoints.append(copy.deepcopy(wpose))

# wpose.position.x += scale * -0.5  # Second move forward/backwards in (x)
# waypoints.append(copy.deepcopy(wpose))

# wpose.position.y -= scale * 0.1  # Third move sideways (y)
# waypoints.append(copy.deepcopy(wpose))

# # We want the Cartesian path to be interpolated at a resolution of 1 cm
# # which is why we will specify 0.01 as the eef_step in Cartesian
# # translation.  We will disable the jump threshold by setting it to 0.0,
# # ignoring the check for infeasible jumps in joint space, which is sufficient
# # for this tutorial.
# (plan, fraction) = move_group.compute_cartesian_path(
#     waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
# )  # jump_threshold

# move_group.execute(plan, wait=True)

# moveit_commander.roscpp_shutdown()


####################
    # def move_end_effector(self, target_pose):
    #     self.group.set_pose_target(target_pose)

    #     while True:
    #         self.group.go()  # Move the end effector

    #         if abs(self.force_error) <= threshold:
    #             break

    #         # Adjust the target pose based on the force error
    #         delta_z = self.force_error * 0.01  # Example adjustment based on the force error
    #         target_pose.position.z += delta_z
    #         self.group.set_pose_target(target_pose)