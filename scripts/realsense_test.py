#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import time
import sys

class ImagePreprocessor:
    def __init__(self, subscribed_topic_name):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(subscribed_topic_name, Image, self.callback)
        self.log_rate = rospy.Rate(1)
        self.store_path = "/home/yunfei/catkin/src/realsense_test/imgs/rgb/"
    
    def callback(self, data):
        self.log_rate.sleep()
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            # cv2.imwrite(self.store_path + str(time.time()) + ".jpg", self.cv_image)
            # rospy.loginfo(self.cv_image.shape)
            self.cv_image = self.cv_image[120:270, 550:700, :]
            cv2.imwrite(self.store_path + "test.jpg", self.cv_image)
            # cv2.imshow("test", self.cv_image)
            # cv2.waitKey(3)
            # self.np_image = np.array(self.cv_image, dtype=np.float32)
            # center_idx = np.array(self.np_image.shape) / 2
            # rospy.loginfo("center depth: %f", self.np_image[center_idx[0], center_idx[1]])
            # rospy.loginfo(data)
            # rospy.loginfo(self.np_image.shape)
        except CvBridgeError as e:
            rospy.logerr(e)

class DepthPreprocessor:
    def __init__(self, subscribed_topic_name):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(subscribed_topic_name, Image, self.callback)
        self.log_rate = rospy.Rate(1)
        self.store_path = "/home/yunfei/catkin/src/realsense_test/imgs/segment/"
    
    def callback(self, data):
        self.log_rate.sleep()
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            self.cv_image = self.cv_image[120:270, 550:700]
            self.segmented_image = np.zeros(self.cv_image.shape)
            base = self.cv_image[0, 0]
            for i in range(self.cv_image.shape[0]):
                for j in range(self.cv_image.shape[1]):
                    if self.cv_image[i, j] > base / 100 and self.cv_image[i, j] < base * 0.8:
                        self.segmented_image[i, j] = 255
            # np.savetxt(self.store_path + "test.txt", self.cv_image)
            # sys.exit()
            # cv2.imshow("test", self.cv_image)
            # cv2.imwrite(self.store_path + str(time.time()) + ".jpg", self.cv_image)
            cv2.imwrite(self.store_path + "test.jpg", self.segmented_image)
        except CvBridgeError as e:
            rospy.logerr(e)

class PointCloudPreprocessor:
    def __init__(self, subscribed_topic_name):
        self.sub = rospy.Subscriber(subscribed_topic_name, PointCloud2, self.callback)
        self.log_rate = rospy.Rate(1)
    
    def callback(self, data):
        self.log_rate.sleep()
        self.data = data
        rospy.loginfo(self.data)
            

if __name__ == "__main__":
    rospy.init_node("realsense_test", log_level=rospy.DEBUG)
    # image_preprocessor = ImagePreprocessor("/camera/color/image_raw")
    depth_prepreprocessor = DepthPreprocessor("/camera/aligned_depth_to_color/image_raw")
    # pointcloud_preprocessor = PointCloudPreprocessor("/camera/depth/color/points")
    rospy.spin()
    
