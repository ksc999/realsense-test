#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import time

class WipingSegmentation:
    def __init__(
            self,
            flag_store_image=False, 
            rgb_store_path=None,
            segmented_store_path=None,
            image_name="test_segment.jpg",
            log_rate=1,
            depth_threshold=None,
            image_cut_info=None
        ):
        self.flag_store_image = flag_store_image
        self.rgb_store_path = rgb_store_path
        self.segmented_store_path = segmented_store_path
        self.image_name = image_name
        self.log_rate = rospy.Rate(log_rate)
        self.depth_threshold = depth_threshold
        self.image_cut_info = image_cut_info
        self.rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        self.rgb_image = None
        self.depth_image = None
        self.segmented_image = None
        self.bridge = CvBridge()
    
    def rgb_callback(self, rgb):
        self.log_rate.sleep()
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
        if self.image_cut_info is not None:
            self.rgb_image = self.rgb_image[
                self.image_cut_info['up']: self.image_cut_info['down'],
                self.image_cut_info['left']: self.image_cut_info['right']
            ]
        if self.flag_store_image:
            self.get_image_name()
            cv2.imwrite(self.rgb_store_path + self.image_name, self.rgb_image)
    
    def depth_callback(self, depth):
        self.log_rate.sleep()
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
        if self.image_cut_info is not None:
            self.depth_image = self.depth_image[
                self.image_cut_info['up']: self.image_cut_info['down'],
                self.image_cut_info['left']: self.image_cut_info['right']
            ]
        if self.rgb_image is not None:
            self.segment()
            if self.flag_store_image:
                self.get_image_name()
                cv2.imwrite(self.segmented_store_path + self.image_name, self.segmented_image)
    
    def get_image_name(self):
        if self.image_name is not None:
            pass
        else:
            self.image_name = str(time.time()) + ".jpg"
    
    def segment(self):
        if self.depth_threshold is not None:
            self.segmented_image = np.sum(self.rgb_image, axis=2) / 3
            self.segmented_image.astype(int)
            flag_rgb = self.segmented_image > 200
            flag_rgb_not = self.segmented_image < 150
            flag1 = self.depth_image > self.depth_threshold['max']
            flag2 = self.depth_image < self.depth_threshold['min']
            flag_depth = flag1 | flag2
            self.segmented_image[flag_rgb] = 255
            self.segmented_image[flag_rgb_not] = 0
            self.segmented_image[flag_depth] = 255
            
            # flag = np.stack([flag_depth for _ in range(3)], axis=2)
            # self.segmented_image = np.copy(self.rgb_image)
            # self.segmented_image[flag] = 255
        else:
            raise Exception('Segmentation method is undefined!')

        
if __name__ == "__main__":
    rospy.init_node("wiping_segment", log_level=rospy.DEBUG)
    image_segmentation = WipingSegmentation(
        flag_store_image=True,
        rgb_store_path="/home/yunfei/catkin/src/realsense_test/imgs/rgb/",
        segmented_store_path="/home/yunfei/catkin/src/realsense_test/imgs/segment/",
        log_rate=5,
        depth_threshold={'max': 1100, 'min': 900},
        image_cut_info={'left': 400, 'right': 800, 'up': 0, 'down': 300},
    )
    rospy.spin()
    
