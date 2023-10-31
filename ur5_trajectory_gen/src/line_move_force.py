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


desired_force = -10.0  # Desired constant force in Newtons -11 had good result with 0.008 gain

class ForceControlNode:
    def __init__(self):
        rospy.init_node('force_control_node')
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("ur5_arm")  # Replace "manipulator" with the name of your UR5 planning group
        self.sub_wrench = rospy.Subscriber('/ft_sensor/raw', WrenchStamped, self.wrench_force_callback)
        self.error_pub =  rospy.Publisher('/force_error', Float64 , queue_size=10)
        self.force_error = 0.0
        self.group.set_max_velocity_scaling_factor(0.5)
        self.home_joint_angles = [-1.50, -1.50, 1.50, -1.55, -1.55, 0.0] 
        self.pick_joint_angles = [-1.4997324032807544, -1.2680472001527, 1.9598007170315341, -2.2199747168046464, -1.5555591206423127, 0.00322782140316491]
        # self.pick_joint_angles = [-1.4999106396561297, -1.2255297314169198, 2.0082943183565654, -2.32396641177049, -1.549817489440592, 0.0007815502676509212] #into table
        # PID constants and variables
        self.Kp = 0.008  # Proportional gain
        self.Ki = 0.005  # Integral gain
        self.Kd = 0.004  # Derivative gain
        self.error_integral = 0.0  # Integral of force error
        self.previous_error = 0.0  # Previous force error
        self.control_effort = 0.0
        self.delta_z = 0.0 

    def wrench_force_callback(self, wrench_msg):
        force_z = wrench_msg.wrench.force.y   #we have force in y
        # self.force_error = force_z  - desired_force
        self.force_error =  desired_force - force_z 

        # PID control
        self.error_integral += self.force_error
        error_derivative = self.force_error - self.previous_error
        self.control_effort = self.Kp * self.force_error + self.Ki * self.error_integral + self.Kd * error_derivative

        # Update previous error
        self.previous_error = self.force_error
        self.force_error = self.control_effort

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
        wpose.position.x += scale * -0.3  # Second move forward/backwards in (x)
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
            # delta_z = self.force_error * -0.008  # Example adjustment based on the force error 0.01  0.008 
            self.delta_z += self.control_effort

            wpose.position.z += self.delta_z
            waypoints[0] = copy.deepcopy(wpose)

            # Add a small delay to allow the robot to respond to the previous adjustment
            time.sleep(0.1)

def main():
    node = ForceControlNode()
    rospy.sleep(0.5)  # Allow time for publishers and subscribers to connect

    node.goto_home_pose() 

    node.goto_pick_pose()

    node.raster_line()

    

if __name__ == '__main__':
    main()

