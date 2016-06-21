#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import turtlesim.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('kinect_calib_listen_tf')

    listener = tf.TransformListener()
    rate = rospy.Rate(10)
    kinect_frame = rospy.get_param('~kinect_frame_name')
    while not rospy.is_shutdown():
        try:
            (trans,quat) = listener.lookupTransform('/base_link', kinect_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            rate.sleep()
            continue
        euler = tf.transformations.euler_from_quaternion(quat)
        # print trans.transform
        print "<origin xyz=\"",
        print trans[0],
        print trans[1],
        print trans[2],
        print "\" rpy=\"",
        print euler[0],
        print euler[1],
        print euler[2],
        print "\"/>"
        
        rate.sleep()
