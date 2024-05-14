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
    ret, frame = cap.read()

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    codec = cap.get(cv2.CAP_PROP_FOURCC)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    # cap.set(cv2.CAP_PROP_FPS, 1)
    fps = cap.get(cv2.CAP_PROP_FPS)
    # target_width, target_height = 426, 240
    target_width, target_height = 100, 100

    rate = rospy.Rate(100)

    print('\nCamera is open:', cap.isOpened())
    print(f"Codec: {decode_fourcc(codec)}")
    print(f"Resolution: {width}x{height}")
    print(f"Frames Per Second (FPS): {fps}\n")

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break
        
        # frame = frame[:, :width // 2, :]

        frame = cv2.resize(frame, (target_width, target_height))

        _, frame = cv2.imencode('.jpg', frame)
        # cv2.imshow('Video', frame)

        # print("Before compress", sys.getsizeof(frame.tobytes()))

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = frame.tobytes()
        pub.publish(msg)

        # size_kb = sys.getsizeof(msg.data) / 1024
        # print("Size of the captured frame: {:.2f} KB".format(size_kb))

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

