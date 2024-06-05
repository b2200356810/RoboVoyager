#!/usr/bin/env python3
import os
import sys
import time
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
# import ultralytics
from ultralytics import YOLO
# from IPython.display import display, Image
import base64

model = YOLO("yolov8n.pt")

def predict(img):
    #img = increase_brightness(img)
    img_resized = cv2.resize(img, (640, 320))
    results = model.predict(img_resized, save=False, imgsz=320, conf=0.3)
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
    return img

def detect(compressed_image_msg):
    np_arr = np.frombuffer(compressed_image_msg.data, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    predicted_image = predict(image)
    
    # Convert image to base64
    _, buffer = cv2.imencode('.jpg', predicted_image)
    # jpg_as_text = base64.b64encode(buffer).decode('utf-8')
    
    pub = rospy.Publisher('/ai_topic', CompressedImage, queue_size=10)
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = buffer.tobytes()
    pub.publish(msg)
    # cv2.namedWindow("Resized_Window", cv2.WINDOW_NORMAL) 
    # cv2.resizeWindow("Resized_Window", 960, 540) 
    # cv2.imshow("Resized_Window", predicted_image)
    # cv2.waitKey(1)

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
    rospy.Subscriber("/video_streaming_topic", CompressedImage, detect)
    rate = rospy.Rate(30)  # 10 Hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        object_detection()
    except rospy.ROSInterruptException:
        pass


"""
def increase_brightness(image, value=20):
    # Convert the image to float32 to prevent overflow
    image = np.clip(image.astype(np.float32) + value, 0, 255).astype(np.uint8)
    return image

def increase_brightness(image, value=30):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # Convert to HSV
    h, s, v = cv2.split(hsv)
    v = cv2.add(v, value)  # Increase brightness
    final_hsv = cv2.merge((h, s, v))
    image = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return image
"""