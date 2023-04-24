#!/usr/bin/env python

import rospy
import tf

rospy.init_node('get_end_effector_position')

# Create a `tf.TransformListener` object to listen for transformations
listener = tf.TransformListener()

# Wait for the transform between the desired frames to become available
listener.waitForTransform('/world', '/ee_link', rospy.Time(), rospy.Duration(4.0))

try:
    # Get the transform from the '/world' frame to the '/end_effector_link' frame
    (trans, rot) = listener.lookupTransform('/world', '/ee_link', rospy.Time(0))
    print("Translation End Effector Position (in world frame):")
    print("X: ", trans[0])
    print("Y: ", trans[1])
    print("Z: ", trans[2])
    print("Rotation")
    print("x: ", rot[0])
    print("y: ", rot[1])
    print("z: ", rot[2])
    print("w: ", rot[3])

except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    rospy.logerr("Failed to get transform from /world to /end_effector_link")
