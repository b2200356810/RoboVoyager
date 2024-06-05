#!/usr/bin/env python3

import sys
import time
import cv2
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

start_time = time.time()

target_width, target_height, target_fps = 800, 500, 30 #Width: 960, Height: 540
new_target_height = target_height
new_target_width = target_width
new_target_fps = target_fps

bridge = CvBridge()
current_image = None

def decode_fourcc(cc):
    return "".join([chr((int(cc) >> 8 * i) & 0xFF) for i in range(4)])

def left_camera_callback(ros_image):
    global current_image
    # Convert ROS Image message to OpenCV image
    try:
        current_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
        height, width = current_image.shape[:2]
        # rospy.loginfo(f"Image received and converted. Width: {width}, Height: {height}")
    except CvBridgeError as e:
        pass
        # rospy.logerr(f"Failed to convert image: {e}")

def image_resize(image, width=None, height=None, inter=cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]

    if width is None and height is None:
        return image

    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))

    resized = cv2.resize(image, dim, interpolation=inter)
    return resized

def streamer():
    global target_width, target_height, target_fps, new_target_width, new_target_height, new_target_fps, current_image
    
    rospy.init_node('video_streaming_node', anonymous=True)
    rospy.Subscriber("/zed2/zed_node/left/image_rect_color", Image, left_camera_callback)
    pub = rospy.Publisher('/video_streaming_topic', CompressedImage, queue_size=1)
    
    rate = rospy.Rate(target_fps)
    
    while not rospy.is_shutdown():
        if current_image is not None:
            # Resize the image if necessary
            # if new_target_width != target_width or new_target_height != target_height:
            #     current_image = image_resize(current_image, width=new_target_width, height=new_target_height)
            # current_image = image_resize(current_image, width=target_width, height=target_height)

            # Encode the image to JPEG
            _, buffer = cv2.imencode('.jpg', current_image)
            
            # Create and publish the compressed image message
            compressed_image_msg = CompressedImage()
            compressed_image_msg.header.stamp = rospy.Time.now()
            compressed_image_msg.format = "jpeg"
            compressed_image_msg.data = buffer.tobytes()
            pub.publish(compressed_image_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        streamer()
    except rospy.ROSInterruptException:
        pass

# For terminal to look up supported codecs and resolutions
# v4l2-ctl -d /dev/video0 --list-formats-ext
# print(cv2.getBuildInformation())


# def left_camera_callback(_ros_image):
#     global target_width, target_height, target_fps, new_target_width, new_target_height, new_target_fps
     
#     rospy.init_node('video_streaming_node')
#     pub = rospy.Publisher('/video_streaming_topic', CompressedImage, queue_size = 10)
#     info_pub = rospy.Publisher("video_streaming_info", VideoStreamInfo, queue_size = 10)
#     rospy.Service("/video_streaming_change_settings", SetVideoStreamSettings, set_new_video_settings)

#     stream_info_msg = VideoStreamInfo()
#     stream_info_msg.codec = "YUYV"
#     stream_info_msg.width = int(target_width)
#     stream_info_msg.height = int(target_height)
#     stream_info_msg.fps = int(target_fps)

#     while not rospy.is_shutdown():
#         loop_start_time = time.time()

#         if target_width != new_target_width or target_height != new_target_height or target_fps != new_target_fps:
#             target_width = new_target_width
#             target_height = new_target_height
#             target_fps = new_target_fps

#             stream_info_msg.width = int(target_width)
#             stream_info_msg.height = int(target_height)
#             stream_info_msg.fps = int(target_fps)

#             print(f"New settings: ")
#             print(f"Resolution: {target_width}x{target_height} @ {target_fps} FPS")
            

#         # frame = frame[:, target_width // 2:, :]
#         # frame = cv2.resize(frame, (target_width, target_height))

#         frame = image_resize(frame, height = target_height)

#         _, frame = cv2.imencode('.jpg', frame)
#         msg = CompressedImage()
#         msg.header.stamp = rospy.Time.now()
#         msg.format = "jpeg"
#         msg.data = frame.tobytes()
#         pub.publish(msg)

#         image_size_kb = round((sys.getsizeof(msg.header.stamp) / 1024) + (sys.getsizeof(msg.format) / 1024) + (sys.getsizeof(msg.data) / 1024), 2)
#         loop_end_time = time.time()
#         loop_duration = loop_end_time - loop_start_time

#         stream_info_msg.image_size_kb = image_size_kb
#         info_pub.publish(stream_info_msg)

#         # print(f"Image size: {image_size_kb} KB")
#         # print(f"Loop time : {loop_duration:.4f} seconds")

#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     end_time = time.time()
#     total_duration = end_time - start_time
#     print(f"Total run time: {total_duration:.4f} seconds\n")
