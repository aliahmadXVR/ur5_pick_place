#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

rospy.init_node('spawn_urdf_model')

# Set the path to the URDF file
model_path = "/home/xavor/.gazebo/models/ccube/ccube.urdf.xacro"

# Read the contents of the URDF file
with open(model_path, "r") as f:
    model_xml = f.read()

# Define the model name and pose
model_name = "my_urdf_model"
model_pose = Pose()
model_pose.position.x = 0.148   # Set the x-coordinate of the position
model_pose.position.y = -0.504   # Set the y-coordinate of the position
model_pose.position.z = 2.0   # Set the z-coordinate of the position

# Create the service proxy for the spawn_model service
spawn_model_proxy = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

# Call the spawn_model service
try:
    resp = spawn_model_proxy(model_name, model_xml, "", model_pose, "world")
    rospy.loginfo("URDF model spawned successfully")
except rospy.ServiceException as e:
    rospy.logerr("Failed to spawn URDF model: %s" % e)
