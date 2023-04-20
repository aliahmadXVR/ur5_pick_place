#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
import select
import termios
import tty

# Initialize MoveIt Commander
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur5_keyboard_control')

robot = moveit_commander.RobotCommander()
group_name = "ur5_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Set the reference frame and end effector link
move_group.set_pose_reference_frame("world")
move_group.set_end_effector_link("ee_link")

# Set the velocity scaling factor
move_group.set_max_velocity_scaling_factor(0.5)

# Define the keyboard control function
def keyboard_control():
    # Set the terminal settings
    settings = termios.tcgetattr(sys.stdin)

    # Set the target pose to the current pose
    current_pose = move_group.get_current_pose().pose
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = current_pose.position.x
    target_pose.position.y = current_pose.position.y
    target_pose.position.z = current_pose.position.z
    target_pose.orientation.x = current_pose.orientation.x
    target_pose.orientation.y = current_pose.orientation.y
    target_pose.orientation.z = current_pose.orientation.z
    target_pose.orientation.w = current_pose.orientation.w

    # Set the motion increments
    dx = 0.03
    dy = 0.03
    dz = 0.02
    dtheta = 0.1

    # Loop until Ctrl-C is pressed
    while not rospy.is_shutdown():
        # Read a key from the terminal
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        # Move the end effector in the desired direction
        if key == 'w':
            target_pose.position.y += dy
        elif key == 's':
            target_pose.position.y -= dy
        elif key == 'a':
            target_pose.position.x -= dx
        elif key == 'd':
            target_pose.position.x += dx
        elif key == 'q':
            target_pose.position.z += dz
        elif key == 'e':
            target_pose.position.z -= dz
        elif key == 'j':
            target_pose.orientation.z += dtheta
        elif key == 'l':
            target_pose.orientation.z -= dtheta
        elif key == 'i':
            target_pose.orientation.x += dtheta
        elif key == 'k':
            target_pose.orientation.x -= dtheta
        elif key == 'u':
            target_pose.orientation.y += dtheta
        elif key == 'o':
            target_pose.orientation.y -= dtheta
        elif key == '\x03':
            break

        # Move the end effector to the target pose
        move_group.set_pose_target(target_pose)
        plan = move_group.plan()
        move_group.execute(plan)

if __name__ == '__main__':
    try:
        keyboard_control()
    except rospy.ROSInterruptException:
        pass

# Clean up MoveIt Commander
moveit_commander.roscpp_shutdown()
