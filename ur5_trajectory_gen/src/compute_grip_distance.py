#!/usr/bin/env python

import math
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState

rospy.init_node('compute_grip_distance')

# Create a proxy for the GetModelState service
get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
# Set the name of the object to get the position of
model_name = 'ccube'

# Create a `tf.TransformListener` object to listen for transformations
listener = tf.TransformListener()
# Create a `tf2_ros.TransformBroadcaster` object to publish transformations
broadcaster = tf2_ros.TransformBroadcaster()

while not rospy.is_shutdown():
    
    # Call the GetModelState service to get the pose of the object
    response = get_model_state(model_name, '')

    # Get the position from the response
    box_position = response.pose.position

    # Wait for the transform between the desired frames to become available
    listener.waitForTransform('/world', '/robotiq_85_left_finger_tip_link', rospy.Time(), rospy.Duration(3.0))

    try:
        # Get the transform from the '/world' frame to the '/end_effector_link' frame
        (finger_trans, finger_rot) = listener.lookupTransform('/world', '/robotiq_85_left_finger_tip_link', rospy.Time(0))

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Failed to get transform from /world to /end_effector_link")

    distance = math.sqrt((box_position.x - finger_trans[0])**2 +
                     (box_position.y - finger_trans[1])**2 +
                     (box_position.z -finger_trans[2])**2)

    print('Distance= ')
    print(distance)

    # If the distance is less than 0.77, publish a transform between the links
    if distance > 0.77:
        transform = tf2_ros.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = '/robotiq_85_left_finger_tip_link'
        transform.child_frame_id = '/ccube'
        transform.transform.translation.x = 0
        transform.transform.translation.y = 0
        transform.transform.translation.z = 0
        transform.transform.rotation.x = 0
        transform.transform.rotation.y = 0
        transform.transform.rotation.z = 0
        transform.transform.rotation.w = 1
        broadcaster.sendTransform(transform)
    # If the distance is greater than or equal to 0.77, remove the transform
    else:
        broadcaster.sendTransform(tf2_ros.TransformStamped())

    rospy.sleep(0.5)




































#------------------------------------------------------
# import math
# import rospy
# import tf
# from geometry_msgs.msg import Pose
# from gazebo_msgs.srv import GetModelState

# rospy.init_node('compute_grip_distance')

# # Create a proxy for the GetModelState service
# get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
# # Set the name of the object to get the position of
# model_name = 'ccube'

# rate = rospy.Rate(2)

# # Create a `tf.TransformListener` object to listen for transformations
# listener = tf.TransformListener()

# # Create a `tf.TransformBroadcaster` object to publish transformations
# broadcaster = tf.TransformBroadcaster()

# while not rospy.is_shutdown():
    
#     # Call the GetModelState service to get the pose of the object
#     response = get_model_state(model_name, '')

#     # Get the position from the response
#     box_position = response.pose.position

#     # Print the position of the object
#     # rospy.loginfo('Position of %s: x=%f, y=%f, z=%f',
#     #               model_name, box_position.x, box_position.y, box_position.z)
#     # Wait for the transform between the desired frames to become available
#     listener.waitForTransform('/world', '/robotiq_85_left_finger_tip_link', rospy.Time(), rospy.Duration(3.0))

#     try:
#         # Get the transform from the '/world' frame to the '/end_effector_link' frame
#         (finger_trans, finger_rot) = listener.lookupTransform('/world', '/robotiq_85_left_finger_tip_link', rospy.Time(0))
#         # print("Left Finger Position (in world frame):")
#         # print("X: ", finger_trans[0])
#         # print("Y: ", finger_trans[1])
#         # print("Z: ", finger_trans[2])
#         # print("-----------------------------")

#     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#         rospy.logerr("Failed to get transform from /world to /end_effector_link")

#     distance = math.sqrt((finger_trans[0] - box_position.x)**2 +
#                         (finger_trans[1] - box_position.y)**2 +
#                         (finger_trans[2] -box_position.z)**2)

#     print('Distance= ')
#     print(distance)

#     if distance < 0.7680:
#         # Define the translation and rotation for the fixed transform
#         fixed_trans = (0.0, 0.0, 0.0)
#         fixed_rot = tf.transformations.quaternion_from_euler(0, 0, 0)

#         # Publish the fixed transform between the finger tip link and the box
#         broadcaster.sendTransform(fixed_trans, fixed_rot, rospy.Time.now(), 'robotiq_85_left_finger_tip_link', model_name)
#         print("tf published")


#     rate.sleep()


# # # Get the positions of the gripper finger and the box
# # gripper_pose = rospy.wait_for_message('/gripper/pose', Pose)
# # box_pose = rospy.wait_for_message('/box/pose', Pose)

# # # Calculate the distance between the gripper finger and the box
# # distance = math.sqrt((box_pose.position.x - gripper_pose.position.x)**2 +
# #                      (box_pose.position.y - gripper_pose.position.y)**2 +
# #                      (box_pose.position.z - gripper_pose.position.z)**2)

# # # Display the distance
# # rospy.loginfo("Distance between gripper finger and box: %f", distance)
