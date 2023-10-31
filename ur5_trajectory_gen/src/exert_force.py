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
import sys
import select
import termios
import tty


desired_force = -5.0  # Desired constant force in Newtons -11 had good result with 0.008 gain

class ForceControlNode:
    def __init__(self):
        rospy.init_node('exert_force')
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("ur5_arm")  # Replace "manipulator" with the name of your UR5 planning group
        self.sub_wrench = rospy.Subscriber('/ft_sensor/raw', WrenchStamped, self.wrench_force_callback)
        self.error_pub =  rospy.Publisher('/force_error', Float64 , queue_size=10)
        self.force_error = 0.0
        self.group.set_max_velocity_scaling_factor(0.5)
        self.home_joint_angles = [-1.50, -1.50, 1.50, -1.55, -1.55, 0.0] 
        # self.pick_joint_angles = [-1.4997324032807544, -1.2680472001527, 1.9598007170315341, -2.2199747168046464, -1.5555591206423127, 0.00322782140316491]
        self.pick_joint_angles = [-1.5010854635968514, -1.248320316100962, 1.9994317106025328, -2.3043189464412084, -1.5459339252547633, 0.0014682085713237925]
        self.delta_z = 0.0 

    def wrench_force_callback(self, wrench_msg):
        force_z = wrench_msg.wrench.force.y   #we have force in y
        self.force_error = force_z  - desired_force
        # self.force_error =  desired_force - force_z 

        # # PID control
        # self.error_integral += self.force_error
        # error_derivative = self.force_error - self.previous_error
        # self.control_effort = self.Kp * self.force_error + self.Ki * self.error_integral + self.Kd * error_derivative

        # # Update previous error
        # self.previous_error = self.force_error
        # self.force_error = self.control_effort

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

    def extert_force(self):
        # Set the target pose to the current pose
        current_pose = self.group.get_current_pose().pose
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = current_pose.position.x
        target_pose.position.y = current_pose.position.y
        target_pose.position.z = current_pose.position.z
        target_pose.orientation.x = current_pose.orientation.x
        target_pose.orientation.y = current_pose.orientation.y
        target_pose.orientation.z = current_pose.orientation.z
        target_pose.orientation.w = current_pose.orientation.w

        # Move the end effector along the desired path while maintaining a constant force
        while not rospy.is_shutdown():
           
            # Adjust the target pose based on the force error
            delta_z = self.force_error * -0.008  # Example adjustment based on the force error 0.01  0.008 
            target_pose.position.z += self.delta_z
            # Move the end effector to the target pose
            self.group.set_pose_target(target_pose)
            plan = self.group.plan()
            self.group.execute(plan)

            # Add a small delay to allow the robot to respond to the previous adjustment
            time.sleep(0.1)

def main():
    node = ForceControlNode()
    rospy.sleep(0.5)  # Allow time for publishers and subscribers to connect

    node.goto_home_pose() 

    node.goto_pick_pose()

    node.extert_force()

    

if __name__ == '__main__':
    main()

