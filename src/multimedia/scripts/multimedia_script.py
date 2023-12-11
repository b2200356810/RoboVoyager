#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

print('\nCamera is open:', cap.isOpened())
print('Resolution: 720x1280')

def talker():
    rospy.init_node('multimedia_node', anonymous=False)
    pub = rospy.Publisher('/multimedia_topic', Image, queue_size=2)
    rate = rospy.Rate(30)

    encoding_format = 'jpg'

    if encoding_format == 'jpg':
        jpg_settings = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
    elif encoding_format == 'png':
        png_settings = [int(cv2.IMWRITE_PNG_COMPRESSION), 5]
    elif encoding_format == 'h264':
        fourcc = cv2.VideoWriter_fourcc(*'H264')
        video_writer = cv2.VideoWriter('output_video.mp4', fourcc, 30.0, (640, 480))

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        if encoding_format == 'jpg':
            _, img_encoded = cv2.imencode('.jpg', frame, jpg_settings)
        elif encoding_format == 'png':
            _, img_encoded = cv2.imencode('.png', frame, png_settings)
        elif encoding_format == 'h264':
            video_writer.write(frame)

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

    if encoding_format == 'h264':
        video_writer.release()
    cap.release()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
