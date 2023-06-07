#!/usr/bin/env python3

import rospy
import numpy as np
import tf
import geometry_msgs.msg


def broadcast_tf():
    # Initialize ROS node
    rospy.init_node('tf_broadcaster')

    # Create a TransformBroadcaster object
    tf_broadcaster = tf.TransformBroadcaster()

    # Set the translation
    translation = [0.05, 0.05, 0.117256]
    x = translation[0]
    y = translation[1]
    z = translation[2]

    # Set the rotation (as a quaternion)

    # Publish the static transform
    tf_broadcaster.sendTransform((x,y,z),(0,0,0,1), rospy.Time.now(),"obj_frame","base_link")
    

if __name__ == '__main__':
    while not rospy.is_shutdown():
        broadcast_tf()
    