#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

cap = cv2.VideoCapture(0)
print('Camera is open: ', cap.isOpened())
bridge = CvBridge()

def talker():
    rospy.init_node('multimedia_node', anonymous=False)
    pub = rospy.Publisher('/multimedia_topic', Image, queue_size=1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        # Terminal: rqt_image_view
        # Encoding for ROS only, not browser 
        msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub.publish(msg)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if rospy.is_shutdown():
            cap.release()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
