#!/usr/bin/env python3

import sys
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
# print(cv2.getBuildInformation())

# For terminal to look up supported codecs and resolutions
# v4l2-ctl -d /dev/video0 --list-formats-ext

def decode_fourcc(cc):
    return "".join([chr((int(cc) >> 8 * i) & 0xFF) for i in range(4)])

def streamer():

    # IMPORTANT! 
    target_width, target_height, target_fps = 1000, 1000, 30

    rospy.init_node('video_streaming_node')
    pub = rospy.Publisher('/video_streaming_topic', CompressedImage, queue_size = 10)

    cap = cv2.VideoCapture(0)
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
        ret, frame = cap.read()
        if not ret:
            break
        
        # Resizing to even smaller resolution
        # frame = frame[:, :width // 2, :]
        frame = cv2.resize(frame, (target_width, target_height))
        _, frame = cv2.imencode('.jpg', frame)
        # cv2.imshow('Video', frame)

        # frame_size = str(round(sys.getsizeof(frame.tobytes()) / 1024, 2))
        # print(f"Before compression: {frame_size} KB")

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = frame.tobytes()
        pub.publish(msg)

        size_kb = str(round(sys.getsizeof(msg.data) / 1024, 2))
        print(f"After  compression: {size_kb} KB")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()
    
    print()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        streamer()
    except rospy.ROSInterruptException:
        pass
