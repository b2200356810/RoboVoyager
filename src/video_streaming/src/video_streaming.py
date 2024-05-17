#!/usr/bin/env python3

import sys
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
# print(cv2.getBuildInformation())

def decode_fourcc(cc):
    return "".join([chr((int(cc) >> 8 * i) & 0xFF) for i in range(4)])

def streamer():

    rospy.init_node('video_streaming_node')
    pub = rospy.Publisher('/video_streaming_topic', CompressedImage, queue_size = 10)

    cap = cv2.VideoCapture(0)
    # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')) # Not supported by Zed2
    codec = cap.get(cv2.CAP_PROP_FOURCC)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1344)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 376)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.set(cv2.CAP_PROP_FPS, 30) # Careful with the fps
    fps = cap.get(cv2.CAP_PROP_FPS)
    rate = rospy.Rate(fps)

    target_width, target_height = 100, 100

    # Open a single frame to get original image size 
    ret, frame = cap.read()
    frame_size = round(sys.getsizeof(frame.tobytes()) / 1024, 2)
    frame = frame[:, :width // 2, :]
    _, frame = cv2.imencode('.jpg', frame)

    print('\nCamera is open:', cap.isOpened())
    print(f"Codec: {decode_fourcc(codec)}")
    print(f"Resolution: {width}x{height}")
    print(f"FPS: {fps}")
    print("Frame size:", str(frame_size) + "KB")

    # Lower resolution and test new image size
    frame = cv2.resize(frame, (target_width, target_height))
    new_height, new_width = frame.shape[:2]
    frame_size = round(sys.getsizeof(frame.tobytes()) / 1024, 2)

    print("After resizing...")
    print(f"Resolution: {new_width}x{new_height}")
    print("Frame size:", frame_size, "KB")
    print()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break
        
        frame = frame[:, :width // 2, :]
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

        # size_kb = str(round(sys.getsizeof(msg.data) / 1024, 2))
        # print(f"After  compression: {size_kb} KB")

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
