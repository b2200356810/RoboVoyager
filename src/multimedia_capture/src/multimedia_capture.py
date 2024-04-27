#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

def capture():
    rospy.init_node('multimedia_capture_node', anonymous=True)
    rospy.loginfo("Initializing camera...")

    # Initialize OpenCV capture
    cap = cv2.VideoCapture(0)  # Use default camera (change as needed)
    if not cap.isOpened():
        rospy.logerr("Failed to open camera.")
        return

    rospy.loginfo("Camera opened successfully.")

    bridge = CvBridge()

    # Create directory for saving images if it doesn't exist
    images_dir = os.path.join(os.path.dirname(__file__), 'images')
    if not os.path.exists(images_dir):
        os.makedirs(images_dir)

    try:
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret:
                # Save image to images directory
                image_path = os.path.join(images_dir, f'image_{rospy.Time.now()}.png')
                cv2.imwrite(image_path, frame)
                rospy.loginfo(f"Image saved: {image_path}")
            else:
                rospy.logwarn("Failed to capture frame from camera.")
    finally:
        # Release the camera capture when done
        cap.release()

if __name__ == '__main__':
    try:
        capture()
    except rospy.ROSInterruptException:
        pass
