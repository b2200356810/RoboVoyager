#!/usr/bin/env python3

import sys
import cv2
import rospy
from sensor_msgs.msg import Image
import time

start_time = time.time()
# print(cv2.getBuildInformation())

# For terminal to look up supported codecs and resolutions
# v4l2-ctl -d /dev/video0 --list-formats-ext

def decode_fourcc(cc):
    return "".join([chr((int(cc) >> 8 * i) & 0xFF) for i in range(4)])

def streamer():

    # IMPORTANT! 
    target_width, target_height, target_fps = 300, 300, 30

    rospy.init_node('video_streaming_node')
    pub = rospy.Publisher('/video_streaming_topic', Image, queue_size = 10)

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')) # Not supported by Zed2
    codec = cap.get(cv2.CAP_PROP_FOURCC)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_height)
    cap.set(cv2.CAP_PROP_FPS, target_fps) # Careful with the fps
    camera_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    camera_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    rate = rospy.Rate(fps)

    print('\nCamera is open:', cap.isOpened())
    print(f"Codec: {decode_fourcc(codec)}")
    print(f"Resolution: {camera_width}x{camera_height}")
    print(f"FPS: {fps}")
    print()

    while not rospy.is_shutdown():
        loop_start_time = time.time()
        ret, frame = cap.read()
        if not ret:
            break
        
        frame = frame[:, :camera_width // 2, :]
        _, frame = cv2.imencode('.jpg', frame)
        frame = cv2.resize(frame, (target_width, target_height))

        img_data = frame
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = 3 * frame.shape[1]
        msg.data = img_data

        size_kb = str(round(sys.getsizeof(msg.data) / 1024, 2))
        # print(f"Image size: {size_kb} KB")

         # Measuring time taken for each iteration
        loop_end_time = time.time()
        loop_duration = loop_end_time - loop_start_time
        print(f"Iteration time: {loop_duration:.4f} seconds")


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

    end_time = time.time()
    total_duration = end_time - start_time
    print(f"Total time: {total_duration:.4f} seconds")
    
    print()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        streamer()
    except rospy.ROSInterruptException:
        pass
