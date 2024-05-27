#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import cv2
import base64
from cv_bridge import CvBridge, CvBridgeError
from http.server import BaseHTTPRequestHandler, HTTPServer
from threading import Thread
import time
from sensor_msgs.msg import Image

bridge = CvBridge()
latest_image = None  # Global variable to store the latest received image

class ImageRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global bridge
        try:
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()

            while True:
                if latest_image is None:
                    time.sleep(0.1)  # Wait for the next image if none is available
                    continue

                # Convert ROS image to OpenCV format
                cv_image = bridge.imgmsg_to_cv2(latest_image, desired_encoding='bgr8')

                # Convert OpenCV image to JPEG format
                _, buffer = cv2.imencode('.jpg', cv_image)
                jpg_bytes = buffer.tobytes()

                # Write the JPEG data to the client
                self.wfile.write(b'--frame\r\n')
                self.send_header('Content-type', 'image/jpeg')
                self.send_header('Content-length', len(jpg_bytes))
                self.end_headers()
                self.wfile.write(jpg_bytes)
                self.wfile.write(b'\r\n')

                time.sleep(0.1)  # Adjust the delay to control the streaming speed
        except Exception as e:
            print(e)

def image_callback(ros_image):
    global latest_image
    latest_image = ros_image

    # Extract resolution
    width = ros_image.width
    height = ros_image.height

    # Extract frame rate (publishing rate)
    publishing_rate = rospy.get_param('/zed/zed_node/left/image_rect_color/publish_rate', default=None)

    rospy.loginfo("Image resolution: {}x{}".format(width, height))
    rospy.loginfo("Publishing rate (fps): {}".format(publishing_rate))

def start_http_server():
    global server
    server = HTTPServer(('0.0.0.0', 8000), ImageRequestHandler)
    server.serve_forever()

def ros_image_streamer():
    rospy.init_node("ros_image_streamer", anonymous=True)
    rospy.Subscriber("/zed/zed_node/left/image_rect_color", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        # Start the HTTP server in a separate thread
        http_thread = Thread(target=start_http_server)
        http_thread.daemon = True
        http_thread.start()

        # Start ROS image streamer
        ros_image_streamer()
    except KeyboardInterrupt:
        if server:
            server.shutdown()
