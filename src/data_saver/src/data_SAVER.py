#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import sys
import os
import cv2
import numpy as np
from datetime import datetime
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, PointField, Imu
import sensor_msgs.point_cloud2 as pc2
from nmea_msgs.msg import Sentence
from tf.msg import tfMessage
# import open3d as o3d
from threading import Thread
from socketserver import ThreadingUnixDatagramServer
from std_msgs.msg import String


bridge = CvBridge()
SAVE = False
FINISH = False
RECORDING_STARTED = False
MAIN_PATH = "/home/emir/recording_tests"
# MAIN_PATH = "/media/moborobot/orin_xtrnl/robovoyager_saved_data"
LEFT_IMG_PATH = None
RIGHT_IMG_PATH = None
DEPTH_PATH = None
PCD_PATH = None
IMU_PATH = None
IMU_FILE = False
TF_PATH = None
TF_FILE = False
GPS_PATH = None
GPS_FILE = False


def left_camera_callback(ros_image):
    global bridge
    global LEFT_IMG_PATH
    if not SAVE:
        return
    try:
        # Convert ros image to cv2 image
        cv_image = bridge.imgmsg_to_cv2(ros_image)
    except CvBridgeError as e:
        print(e)
    timestr = datetime.now().strftime(
        "%Y_%m_%d-%H_%M_%S_%f")[:-3]  # Take current time


    header_time = str(ros_image.header.stamp.to_time())
    file_name = "image-"+header_time+".png"  # Generate file name according to time

    if not LEFT_IMG_PATH == None:
        file_path = os.path.join(LEFT_IMG_PATH, file_name)
    else:
        return
    cv2.imwrite(file_path, cv_image)

def right_camera_callback(ros_image):
    global bridge
    global RIGHT_IMG_PATH
    if not SAVE:
        return
    try:
        # Convert ros image to cv2 image
        cv_image = bridge.imgmsg_to_cv2(ros_image)
    except CvBridgeError as e:
        print(e)
    
    header_time = str(ros_image.header.stamp.to_time())
    timestr = datetime.now().strftime(
        "%Y_%m_%d-%H_%M_%S_%f")[:-3]  # Take current time
    file_name = "image-"+header_time+".png"  # Generate file name according to time
    if not RIGHT_IMG_PATH == None:
        file_path = os.path.join(RIGHT_IMG_PATH, file_name)
    else:
        return
    cv2.imwrite(file_path, cv_image)

def depth_callback(depth_image):
    global bridge
    if not SAVE:
        return
    try:
        # Convert ros image to cv2 image
        cv_image = bridge.imgmsg_to_cv2(depth_image)
    except CvBridgeError as e:
        print(e)
    
    timestr = datetime.now().strftime(
        "%Y_%m_%d-%H_%M_%S_%f")[:-3]  # Take current time

    header_time = str(depth_image.header.stamp.to_time())

    # Generate file name according to time
    file_name = "depth_image-"+header_time+".png"
    if not DEPTH_PATH == None:
        file_path = os.path.join(DEPTH_PATH, file_name)
    else:
        return
    # Convert cv2 image to numpy array in np.float32 data type
    depth_array = np.array(cv_image, dtype=np.float32)
    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
        cv_image, alpha=255/depth_array.max()), cv2.COLORMAP_BONE)
    cv2.imwrite(file_path, depth_colormap)

def lidar_callback(ros_cloud):
    if not SAVE:
        return
    # Get cloud data from ros_cloud
    field_names = [field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(
        ros_cloud, skip_nans=True, field_names=field_names))

    # Check empty
    open3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data) == 0:
        print("Converting an empty cloud")
        return None

    xyz = [(x, y, z) for x, y, z, i in cloud_data]  # get xyzi
    i = [(i) for x, y, z, i in cloud_data]  # get xyzi
    open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
    i = np.array(i).reshape(-1, 1)
    i = np.concatenate((i, i, i), axis=1)

    open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(i))
    timestr = datetime.now().strftime(
        "%Y_%m_%d-%H_%M_%S_%f")[:-3]  # Take current time


    header_time = str(ros_cloud.header.stamp.to_time())
    file_name = "lidar-"+header_time+".pcd"     # Generate file name according to time
    if not PCD_PATH == None:
        file_path = os.path.join(PCD_PATH, file_name)
    else:
        return
    o3d.io.write_point_cloud(file_path, open3d_cloud)

def imu_callback(imu_data):
    global IMU_PATH
    global IMU_FILE
    if not SAVE:
        return
    if not IMU_PATH == None:
        file_name = "imu.txt"       # Generate file name
        file_path = os.path.join(IMU_PATH, file_name)
        timestr = datetime.now().strftime(
            "%Y_%m_%d-%H_%M_%S_%f")[:-3]  # Take current time

        header_time = str(imu_data.header.stamp.to_time())
        if not IMU_FILE:
            txt_file = open(file_path, 'w')
            IMU_FILE = True
        else:
            txt_file = open(file_path, 'a')
        data = "Time: " + header_time+"\n" + \
            str(imu_data)+"\n**********************************************************************\n"
        txt_file.write(data)
        txt_file.close()
    else:
        return

def tf_callback(tf_data):
    global TF_PATH
    global TF_FILE
    if not SAVE or TF_PATH == None:
        return
    timestr = datetime.now().strftime(
        "%Y_%m_%d-%H_%M_%S_%f")[:-3]  # Take current time
    
    # header_time = str(tf_data.header.stamp.to_time())
    file_name = "tf.txt"        # Generate file name according to time
    file_path = os.path.join(TF_PATH, file_name)
    if not TF_FILE:
        txt_file = open(file_path, 'w')
        TF_FILE = True
    else:
        txt_file = open(file_path, 'a')
    data = "Time: "+timestr+"\n" + \
        str(tf_data)+"\n**********************************************************************\n"
    txt_file.write(data)
    txt_file.close()

def gps_callback(gps_data):
    global GPS_PATH
    global GPS_FILE
    if not SAVE or GPS_PATH == None:
        return
    '''Callback function of subscribed topic. '''
    header_time = str(gps_data.header.stamp.to_time())
    sentence = gps_data.sentence
    #header_time = str(rospy.Time.now().to_time())

    timestr = datetime.now().strftime(
        "%Y_%m_%d-%H_%M_%S_%f")[:-3]  # Take current time
    file_name = "gps.txt"        # Generate file name according to time
    file_path = os.path.join(GPS_PATH, file_name)

    if not GPS_FILE:
        myfile = open(file_path, 'w')
        GPS_FILE = True
    else:
        myfile = open(file_path, 'a')

    myfile.write("New GPS Data:\n")
    myfile.write(header_time)
    myfile.write("\n")
    myfile.write(sentence)
    myfile.write("\n")



def ros_left_image():
    rospy.Subscriber("/zed2/zed_node/left/image_rect_color", Image, left_camera_callback)
    rospy.spin()

def ros_right_image():
    rospy.Subscriber("/zed2/zed_node/right/image_rect_color", Image, right_camera_callback)
    rospy.spin()

def ros_depth_image():
    rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, depth_callback)
    rospy.spin()

def ros_lidar():
    rospy.Subscriber("/rslidar_points", PointCloud2, lidar_callback)
    rospy.spin()

def ros_imu():
    rospy.Subscriber("/zed2/zed_node/imu/data", Imu, imu_callback)
    rospy.spin()

def ros_tf():
    rospy.Subscriber("/tf", tfMessage, tf_callback)
    rospy.spin()

def ros_gps():
    gps_sub = rospy.Subscriber("/gps", Sentence, gps_callback)
    rospy.spin()


def take_input(_message):
    global SAVE, FINISH, RECORDING_STARTED
    global LEFT_IMG_PATH, RIGHT_IMG_PATH, DEPTH_PATH, PCD_PATH
    global IMU_PATH, IMU_FILE, TF_PATH, TF_FILE, GPS_PATH, GPS_FILE

    my_input = _message.data

    if my_input == 'start':
        if not RECORDING_STARTED: 
            dir_count = len(os.listdir(MAIN_PATH))
            timestr = datetime.now().strftime("%Y_%m_%d")
            folder_name = str(dir_count + 1) + "-" + timestr
            CURRENT_FOLDER = os.path.join(MAIN_PATH, folder_name)
            os.mkdir(CURRENT_FOLDER)

            LEFT_IMG_PATH = os.path.join(CURRENT_FOLDER, "left_images")
            RIGHT_IMG_PATH = os.path.join(CURRENT_FOLDER, "right_images")
            DEPTH_PATH = os.path.join(CURRENT_FOLDER, "depth")
            PCD_PATH = os.path.join(CURRENT_FOLDER, "pcd")
            IMU_PATH = os.path.join(CURRENT_FOLDER, "imu")
            TF_PATH = os.path.join(CURRENT_FOLDER, "tf")
            GPS_PATH = os.path.join(CURRENT_FOLDER, "gps")

            os.mkdir(LEFT_IMG_PATH)
            os.mkdir(RIGHT_IMG_PATH)
            os.mkdir(DEPTH_PATH)
            os.mkdir(PCD_PATH)
            os.mkdir(IMU_PATH)
            os.mkdir(TF_PATH)
            os.mkdir(GPS_PATH)

            RECORDING_STARTED = True
            print("Recording has started")
        
        SAVE = True
        IMU_FILE = False
        TF_FILE = False
        GPS_FILE = False

    elif my_input == 'pause':
        if SAVE:
            print("Recording is paused")
            SAVE = False 
        else:
            print("Recording is already paused")

    elif my_input == 'stop':
        SAVE = False
        FINISH = True
        RECORDING_STARTED = False
        print("Recording saved")


def main():
    rospy.init_node("data_saver_node", anonymous=True)
    rospy.Subscriber("/data_saver_control", String, take_input)
    
    t1 = Thread(target=ros_left_image)
    t2 = Thread(target=ros_right_image)
    t3 = Thread(target=ros_depth_image)
    t4 = Thread(target=ros_lidar)
    t5 = Thread(target=ros_imu)
    t6 = Thread(target=ros_tf)
    t7 = Thread(target=take_input)
    t8 = Thread(target=ros_gps)

    t1.start()
    t2.start()
    t3.start()
    t4.start()
    t5.start()
    t6.start()
    t7.start()
    t8.start()

    t1.join()
    t2.join()
    t3.join()
    t4.join()
    t5.join()
    t6.join()
    t7.join()
    t8.join()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass