import os
HOME = os.getcwd()
from IPython import display
import ultralytics
import cv2
from ultralytics import YOLO
from IPython.display import display, Image

def predict(img):
    model = YOLO("yolov8n.pt")
    results = model.predict(img, save=False, imgsz=320, conf=0.6)
    for result in results:
        boxes = result.boxes.xyxy  # xyxy format: x1, y1, x2, y2
        img = result.orig_img
        # Draw the bounding boxes on the image
        for box in boxes:
            x1, y1, x2, y2 = box
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            # cv2.putText(img, f"Class: {box['class_id']}", (box["x"], box["y"] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5)
    cv2.imshow("Predicted Image", img)
    cv2.waitKey(0)
    return img

predict(HOME +"/image-2022_08_02-16_30_23_725.png")