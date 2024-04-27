#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class WebInterfaceController:
    def __init__(self):
        rospy.init_node('controls_node')

        # Publisher to command the robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribe to the joystick topic
        rospy.Subscriber('/controls_topic', Joy, self.joy_callback)

    def joy_callback(self, data):
        # Print received joystick commands
        print("Received joystick commands:")
        print("Axes:", data.axes)
        print("Buttons:", data.buttons)

        # Process joystick commands and send velocity commands to move the robot
        twist = Twist()
        twist.linear.x = data.axes[1]  # Assuming forward/backward movement
        twist.angular.z = data.axes[0]  # Assuming rotation
        self.cmd_vel_pub.publish(twist)

def main():
    controller = WebInterfaceController()
    rospy.spin()

if __name__ == '__main__':
    main()
