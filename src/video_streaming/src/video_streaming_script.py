#!/usr/bin/env python3

import sys
import time
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from video_streaming.srv import SetVideoStreamSettings, SetVideoStreamSettingsResponse
from video_streaming.msg import VideoStreamInfo

start_time = time.time()

target_width, target_height, target_fps = 1344, 100, 30
# target_width, target_height, target_fps = 2560, 720, 30
new_target_height = target_height
new_target_width = target_width
new_target_fps = target_fps

def set_new_video_settings(req):
    global new_target_width, new_target_height, new_target_fps
    new_target_width = req.width
    new_target_height = req.height
    new_target_fps = req. fps
    return SetVideoStreamSettingsResponse(success = True, message = f"New settings: {new_target_width} x {new_target_height} @ {new_target_fps} FPS")

def streamer():

    global target_width, target_height, target_fps, new_target_width, new_target_height, new_target_fps

    rospy.init_node('video_streaming_node')
    pub = rospy.Publisher('/video_streaming_topic', CompressedImage, queue_size = 10)
    info_pub = rospy.Publisher("video_streaming_info", VideoStreamInfo, queue_size = 10)
    rospy.Service("/video_streaming_change_settings", SetVideoStreamSettings, set_new_video_settings)

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')) # Not supported by Zed2
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_height)
    cap.set(cv2.CAP_PROP_FPS, target_fps)
    codec = decode_fourcc(cap.get(cv2.CAP_PROP_FOURCC))
    target_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    target_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    target_fps = int(cap.get(cv2.CAP_PROP_FPS))
    rate = rospy.Rate(target_fps) # Looping faster than streaming doesn't stream images faster

    stream_info_msg = VideoStreamInfo()
    stream_info_msg.codec = codec
    stream_info_msg.width = int(target_width)
    stream_info_msg.height = int(target_height)
    stream_info_msg.fps = int(target_fps)

    print('\nCamera is open:', cap.isOpened())
    print(f"Codec: {codec}")
    print(f"Resolution: {target_width}x{target_height} @ {target_fps} FPS\n")

    while not rospy.is_shutdown():
        loop_start_time = time.time()
        ret, frame = cap.read()
        if not ret:
            break

        if target_width != new_target_width or target_height != new_target_height or target_fps != new_target_fps:
            target_width = new_target_width
            target_height = new_target_height
            target_fps = new_target_fps

            stream_info_msg.width = int(target_width)
            stream_info_msg.height = int(target_height)
            stream_info_msg.fps = int(target_fps)

            rate = rospy.Rate(target_fps)
            # cap.set(cv2.CAP_PROP_FPS, target_fps)

            print(f"New settings: ")
            print(f"Resolution: {target_width}x{target_height} @ {target_fps} FPS")
            

        # Cut off the right camera
        frame = frame[:, target_width // 2:, :]
        # frame = cv2.resize(frame, (target_width, target_height))

        frame = image_resize(frame, height = target_height)

        _, frame = cv2.imencode('.jpg', frame)
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = frame.tobytes()
        pub.publish(msg)

        image_size_kb = round((sys.getsizeof(msg.header.stamp) / 1024) + (sys.getsizeof(msg.format) / 1024) + (sys.getsizeof(msg.data) / 1024), 2)
        loop_end_time = time.time()
        loop_duration = loop_end_time - loop_start_time

        stream_info_msg.image_size_kb = image_size_kb
        info_pub.publish(stream_info_msg)

        # print(f"Image size: {image_size_kb} KB")
        # print(f"Loop time : {loop_duration:.4f} seconds")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

    end_time = time.time()
    total_duration = end_time - start_time
    print(f"Total run time: {total_duration:.4f} seconds\n")

    cap.release()
    cv2.destroyAllWindows()

def decode_fourcc(cc):
    return "".join([chr((int(cc) >> 8 * i) & 0xFF) for i in range(4)])

def image_resize(image, width = None, height = None, inter = cv2.INTER_AREA):
    # initialize the dimensions of the image to be resized and
    # grab the image size
    dim = None
    (h, w) = image.shape[:2]

    # if both the width and height are None, then return the
    # original image
    if width is None and height is None:
        return image

    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the
        # dimensions
        r = height / float(h)
        dim = (int(w * r), height)

    # otherwise, the height is None
    else:
        # calculate the ratio of the width and construct the
        # dimensions
        r = width / float(w)
        dim = (width, int(h * r))

    # resize the image
    resized = cv2.resize(image, dim, interpolation = inter)

    # return the resized image
    return resized


if __name__ == '__main__':
    try:
        streamer()
    except rospy.ROSInterruptException:
        pass

# For terminal to look up supported codecs and resolutions
# v4l2-ctl -d /dev/video0 --list-formats-ext
# print(cv2.getBuildInformation())