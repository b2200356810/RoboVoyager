#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
import torch
import torchvision.transforms as T 
from torchvision import models 
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage  # Alias to avoid confusion
import matplotlib.pyplot as plt
import time

bridge = CvBridge()
fcn = models.segmentation.fcn_resnet50(weights='FCN_ResNet50_Weights.COCO_WITH_VOC_LABELS_V1').eval()
trf = T.Compose([T.Resize(256), T.ToTensor(), T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])])

def decode_segmap(image, nc=21):
    label_colors = np.array([(0, 0, 0),  # 0=background
                             # 1=aeroplane, 2=bicycle, 3=bird, 4=boat, 5=bottle
                             (128, 0, 0), (0, 128, 0), (128, 128, 0), (0, 0, 128), (128, 0, 128),
                             # 6=bus, 7=car, 8=cat, 9=chair, 10=cow
                             (0, 128, 128), (128, 128, 128), (64, 0, 0), (192, 0, 0), (64, 128, 0),
                             # 11=dining table, 12=dog, 13=horse, 14=motorbike, 15=person
                             (192, 128, 0), (64, 0, 128), (192, 0, 128), (64, 128, 128), (192, 128, 128),
                             # 16=potted plant, 17=sheep, 18=sofa, 19=train, 20=tv/monitor
                             (0, 64, 0), (128, 64, 0), (0, 192, 0), (128, 192, 0), (0, 64, 128)])
    r = np.zeros_like(image).astype(np.uint8)
    g = np.zeros_like(image).astype(np.uint8)
    b = np.zeros_like(image).astype(np.uint8)
    for l in range(0, nc):
        idx = image == l
        r[idx] = label_colors[l, 0]
        g[idx] = label_colors[l, 1]
        b[idx] = label_colors[l, 2]
    rgb = np.stack([r, g, b], axis=2)
    return rgb

def segment(ros_image):
    start_time = time.time()  # Start time for performance measurement

    try:
        current_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='rgb8')
        current_image = cv2.resize(current_image, (300, 250))
    except CvBridgeError as e:
        print(e)
        return
    
    # Convert the image to PIL format for transformations
    img_pil = PILImage.fromarray(current_image)
    
    # Perform the segmentation
    with torch.no_grad():
        inp = trf(img_pil).unsqueeze(0)
        out = fcn(inp)["out"]

    om = torch.argmax(out.squeeze(), dim=0).detach().cpu().numpy()
    rgb = decode_segmap(om) 

    segmentation_overlay = PILImage.fromarray(rgb, 'RGB')
    segmentation_overlay = segmentation_overlay.resize(img_pil.size)

    alpha = 0.5
    combined_image = PILImage.blend(img_pil, segmentation_overlay, alpha)
    combined_image = np.array(combined_image)

    # cv2.namedWindow("Resized_Window", cv2.WINDOW_NORMAL) 
    # cv2.resizeWindow("Resized_Window", 960, 540) 
    # cv2.imshow("Resized_Window", combined_image)
    # cv2.waitKey(1)

    _, buffer = cv2.imencode('.jpg', combined_image)
    pub = rospy.Publisher('/segmentation_streaming_topic', CompressedImage, queue_size=10)
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = buffer.tobytes()
    pub.publish(msg)

    end_time = time.time()  # End time for performance measurement
    print(f"Segmentation processing time: {end_time - start_time:.2f} seconds")

    # _, buffer = cv2.imencode('.jpg', predicted_image)
    # pub = rospy.Publisher('/ai_streaming_topic', CompressedImage, queue_size=10)
    # msg = CompressedImage()
    # msg.header.stamp = rospy.Time.now()
    # msg.format = "jpeg"
    # msg.data = buffer.tobytes()
    # pub.publish(msg)



def segmentation():
    rospy.init_node('segmentation_node')
    rospy.Subscriber("/zed2/zed_node/right/image_rect_color", Image, segment)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        segmentation()
    except rospy.ROSInterruptException:
        pass
