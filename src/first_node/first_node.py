#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    received_message = data.data
    concatenated_message = "First node: " + received_message
    rospy.loginfo("Received: %s", received_message)
    rospy.loginfo("Publishing: %s", concatenated_message)
    pub = rospy.Publisher('first_node_topic', String, queue_size=10)
    pub.publish(concatenated_message)

def listener():
    rospy.init_node('first_node', log_level=rospy.DEBUG, anonymous=True)
    rospy.Subscriber('original_topic', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()