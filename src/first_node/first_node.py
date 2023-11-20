#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    received_message = data.data
    concatenated_message = "First node: " + received_message
    rospy.loginfo("Received: %s", received_message)
    rospy.loginfo("Publishing: %s", concatenated_message)
    pub = rospy.Publisher('/first_node_topic', String, queue_size=10)
    pub.publish(concatenated_message)

def hello_publisher():
    rospy.init_node('first_node', log_level=rospy.DEBUG, anonymous=True)
    rospy.Subscriber('original_topic', String, callback)
    rate = rospy.Rate(1)  # 1 Hz, i.e., publish every 1 second

    while not rospy.is_shutdown():
        pub = rospy.Publisher('/first_node_topic', String, queue_size=10)
        pub.publish("Hello world")
        rate.sleep()

if __name__ == '__main__':
    try:
        hello_publisher()
    except rospy.ROSInterruptException:
        pass
