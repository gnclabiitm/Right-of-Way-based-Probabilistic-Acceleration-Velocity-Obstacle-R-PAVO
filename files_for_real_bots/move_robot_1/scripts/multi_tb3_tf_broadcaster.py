#!/usr/bin/env python

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('multi_tb3_tf_broadcaster')

    # Define the transform from odom to tb3_0/base_footprint
    tb3_0_broadcaster = tf.TransformBroadcaster()
    tb3_0_broadcaster.sendTransform(
        (0, 0, 0),  # Translation
        tf.transformations.quaternion_from_euler(0, 0, 0),  # Rotation
        rospy.Time.now(),  # Time
        'tb3_0/base_footprint',  # Child frame
        'odom'  # Parent frame
    )

    # Define the transform from odom to tb3_1/base_footprint
    tb3_1_broadcaster = tf.TransformBroadcaster()
    tb3_1_broadcaster.sendTransform(
        (0, 0, 0),  # Translation
        tf.transformations.quaternion_from_euler(0, 0, 0),  # Rotation
        rospy.Time.now(),  # Time
        'tb3_1/base_footprint',  # Child frame
        'odom'  # Parent frame
    )

    # # Define the transform from odom to tb3_2/base_footprint
    # tb3_2_broadcaster = tf.TransformBroadcaster()
    # tb3_2_broadcaster.sendTransform(
    #     (0, 0, 0),  # Translation
    #     tf.transformations.quaternion_from_euler(0, 0, 0),  # Rotation
    #     rospy.Time.now(),  # Time
    #     'tb3_1/base_footprint',  # Child frame
    #     'odom'  # Parent frame
    # )

    rospy.spin()
