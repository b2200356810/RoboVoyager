#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
import torch
import torchvision.transforms as T 
from torchvision import models 
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image 
import matplotlib.pyplot as plt 



fcn = models.segmentation.fcn_resnet50(pretrained=True).eval()
img = Image.open("/home/moborobot/RoboVoyager/src/yolo/image-2022_08_04-15_52_48_862.png") 
img = img.convert("RGB")
trf = T.Compose([T.Resize(256),  T.ToTensor(), T.Normalize(mean = [0.485, 0.456, 0.406], std = [0.229, 0.224, 0.225])]) 
inp = trf(img).unsqueeze(0)
out = fcn(inp)["out"] 
om = torch.argmax(out.squeeze(), dim=0).detach().cpu().numpy() 

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


rgb = decode_segmap(om) 

cv2.imshow('window_name', rgb) 
  
cv2.waitKey(0) 
cv2.destroyAllWindows() 
