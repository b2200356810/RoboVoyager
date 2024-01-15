#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def hello_publisher():
    rospy.init_node('hello_world_node', anonymous=True)
    pub = rospy.Publisher('/hello_world_topic', String, queue_size=10)
    rate = rospy.Rate(1/2)

    while not rospy.is_shutdown():
        # hello_string = "hello world %s" % rospy.get_time()
        hello_string = "hello world"
        pub.publish(hello_string)
        rate.sleep()

if __name__ == '__main__':
    try:
        hello_publisher()
    except rospy.ROSInterruptException:
        pass
