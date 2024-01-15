#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image

# print(cv2.getBuildInformation())

encoding_format = 'jpg'

if encoding_format == 'jpg':
    jpg_settings = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
elif encoding_format == 'png':
    png_settings = [int(cv2.IMWRITE_PNG_COMPRESSION), 2]
    
def decode_fourcc(cc):
    return "".join([chr((int(cc) >> 8 * i) & 0xFF) for i in range(4)])

def talker():
    rospy.init_node('video_streaming_node', anonymous=False)
    pub = rospy.Publisher('/video_streaming_topic', Image, queue_size = 5)
    rate = rospy.Rate(30)

    cap = cv2.VideoCapture(2)
    print('\nCamera is open:', cap.isOpened())
    codec = cap.get(cv2.CAP_PROP_FOURCC)
    print(f"Codec: {decode_fourcc(codec)}")
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)

    print(f"Encoding selected: {encoding_format}")

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Resolution: {width}x{height}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Frames Per Second (FPS): {fps}")

    ret, frame = cap.read()
    size_kb = len(cv2.imencode('.jpg', frame)[1]) / 1000
    print(f"Frame Size: {size_kb:.2f} KB")

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        # cv2.imshow('Video', frame)

        if encoding_format == 'jpg':
            _, img_encoded = cv2.imencode('.jpg', frame, jpg_settings)
        elif encoding_format == 'png':
            _, img_encoded = cv2.imencode('.png', frame, png_settings)

        img_data = img_encoded.tobytes()
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = 3 * frame.shape[1]
        msg.data = img_data

        pub.publish(msg)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass