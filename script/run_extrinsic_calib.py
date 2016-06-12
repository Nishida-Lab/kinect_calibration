#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2

import rospy
# Camera
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from kinect_calibration.extrinsic_calibration import ExtrinsicCalibration
# TF
import tf
import tf2_ros
import geometry_msgs.msg

class getTFbyExtrinsicCalib(object):
    
    def __init__(self):
        self.caminfo_sub_topic = rospy.get_param('~cam_info_topic')
        self.image_sub_topic = rospy.get_param('~image_topic')
        self.frame = rospy.get_param('~frame_name')
        self.object_frame = rospy.get_param('~object_frame_name')
        self.row = rospy.get_param('~row_num')
        self.column = rospy.get_param('~column_num')
        self.size = rospy.get_param('~size')
        self.target_type = rospy.get_param('~target_type')

        self.ex_calib = ExtrinsicCalibration(self.row, self.column, self.size, None, None)
        self.bridge = CvBridge()
        
        image_sub = rospy.Subscriber(self.image_sub_topic, Image, self.imageCallback)
        info_sub = rospy.Subscriber(self.caminfo_sub_topic, CameraInfo, self.camInfoCallback)
    
    def imageCallback(self, message):
        img = self.bridge.imgmsg_to_cv2(message, "bgr8")
        ret = self.ex_calib.calibrate(img, self.target_type)
        if ret is True :
            br = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.object_frame
            t.child_frame_id = self.frame

            t.transform.translation.x = self.ex_calib.inv_tvecs[0][0]
            t.transform.translation.y = self.ex_calib.inv_tvecs[1][0]
            t.transform.translation.z = self.ex_calib.inv_tvecs[2][0]
            q = tf.transformations.quaternion_from_euler(self.ex_calib.inv_roll, self.ex_calib.inv_pitch, self.ex_calib.inv_yaw)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            br.sendTransform(t)
            rospy.loginfo("Get a Image")
            
        else :
            rospy.logwarn("Failed to get Pose")
            
        k = self.ex_calib.imgShow(img, ret)
        
    def camInfoCallback(self, message):
        mtx = np.float64([[message.K[0], message.K[1], message.K[2]],
                          [message.K[3], message.K[4], message.K[5]],
                          [message.K[6], message.K[7], message.K[8]]])
        dist = list(message.D)
        self.ex_calib.mtx = mtx
        self.ex_calib.dist = np.float64(dist)
    
if __name__ == "__main__":
        
    rospy.init_node('camera_extrinsic_calibrater')
    node = getTFbyExtrinsicCalib()
    rospy.spin()
