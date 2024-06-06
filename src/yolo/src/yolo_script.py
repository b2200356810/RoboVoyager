#!/usr/bin/env python3

import os
import sys
import time
import cv2
import rospy
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge, CvBridgeError
import torch

# Check if CUDA is available and set the device accordingly
device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f"Using device: {device}")

model = YOLO("/home/moborobot/RoboVoyager/src/yolo/best.pt").to(device)
#model = YOLO("yolov8n.pt")
bridge = CvBridge()

def predict(img):
    #img = increase_brightness(img)
    results = model.predict(img, save=False, imgsz=320, conf=0.4)
    for result in results:
        boxes = result.boxes.xyxy  # xyxy format: x1, y1, x2, y2
        class_ids = result.boxes.cls  # Get the class IDs
        img = result.orig_img
        labels = result.names  # Get the class labels
        # Draw the bounding boxes on the image
        for box, class_id in zip(boxes, class_ids):
            x1, y1, x2, y2 = box
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 1)
            label = labels[int(class_id)]  # Get the class label
            cv2.putText(img, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_TRIPLEX, 0.3, (0, 0, 255), 1,cv2.LINE_AA)
            # print(f"Box: {box} - Label: {label}")
        height, width = img.shape[:2]
        print(f"Image: Width: {width}, Height: {height}")
    return img

def detect(ros_image):
    current_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
    # img_resized = cv2.resize(current_image, (200, 320))
    predicted_image = predict(current_image)


    cv2.namedWindow("Resized_Window", cv2.WINDOW_NORMAL) 
    cv2.resizeWindow("Resized_Window", 960, 540) 
    cv2.imshow("Resized_Window", predicted_image)
    cv2.waitKey(1)

    _, buffer = cv2.imencode('.jpg', predicted_image)
    pub = rospy.Publisher('/ai_streaming_topic', CompressedImage, queue_size=10)
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = buffer.tobytes()
    pub.publish(msg)
   

def adjust_brightness(image, brightness_factor):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    v = cv2.add(v, brightness_factor)
    v = np.clip(v, 0, 255)
    final_hsv = cv2.merge((h, s, v))
    image = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return image

def object_detection():
    rospy.init_node('yolo_node')
    rospy.Subscriber("/zed2/zed_node/right/image_rect_color", Image, detect)
    rate = rospy.Rate(30)  # 10 Hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        object_detection()
    except rospy.ROSInterruptException:
        pass
