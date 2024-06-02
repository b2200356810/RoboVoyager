#!/usr/bin/env python3

import os
import sys
import time
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np

from IPython import display

import ultralytics
from ultralytics import YOLO
from IPython.display import display, Image

model = YOLO("yolov8n.pt")


def predict(img):
    results = model.predict(img, save=False, imgsz=320, conf=0.6)
    for result in results:
        boxes = result.boxes.xyxy  # xyxy format: x1, y1, x2, y2
        img = result.orig_img
        labels = result.names  # Get the class labels
        # Draw the bounding boxes on the image
        for box in boxes:
            x1, y1, x2, y2 = box
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            label = labels[int(box[-1])]  # Assuming box[-1] is the class ID
            cv2.putText(img, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return img

def detect(compressed_image_msg):
    # Convert compressed image data to numpy array
    np_arr = np.frombuffer(compressed_image_msg.data, np.uint8)
    # Decode the image
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    # Display the image
    predicted_image = predict(image)
    # Display the image
    cv2.imshow("Predicted Image", predicted_image)
    cv2.waitKey(1)
    

def object_detection():

    rospy.init_node('yolo_node')
    rospy.Subscriber("/video_streaming_topic", CompressedImage, detect)
    rospy.spin()

if __name__ == '__main__':
    try:
        object_detection()
    except rospy.ROSInterruptException:
        pass
