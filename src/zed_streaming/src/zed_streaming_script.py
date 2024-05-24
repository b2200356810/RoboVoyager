#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import sys
import os
import cv2
from datetime import datetime
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from threading import Thread

bridge = CvBridge()

# --Time formats
#timestr = datetime.now().strftime("%Y_%m_%d-%H_%M_%S_%f")[:-3]
#timestr = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")


#### CALLBACK FUNCTIONS ####
def left_camera_callback(ros_image):
    global bridge
    try:
        # Convert ros image to cv2 image
        cv_image = bridge.imgmsg_to_cv2(ros_image)
    except CvBridgeError as e:
        print(e)
    timestr = datetime.now().strftime(
        "%Y_%m_%d-%H_%M_%S_%f")[:-3]  # Take current time


    header_time = str(ros_image.header.stamp.to_time())
    file_name = "image-"+header_time+".png"  # Generate file name according to time

    pub = rospy.Publisher('/zed_test_pub', Image, queue_size = 10)
    pub.publish(cv_image)

def ros_left_image():
    rospy.Subscriber("/zed2/zed_node/left/image_rect_color",
                     Image, left_camera_callback)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node("zed_streamer_node", anonymous=True)
    t1 = Thread(target=ros_left_image)
    t1.start()
    t1.join()