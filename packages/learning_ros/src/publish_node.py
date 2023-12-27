#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

def move_turtle_square():
    rospy.init_node('move_turtle_square', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtlesim/turtle1/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(1)

    time.sleep(1)

    while not rospy.is_shutdown():

        forward_msg = Twist()
        forward_msg.linear.x = 2.0

        velocity_publisher.publish(forward_msg)

        time.sleep(5)

        turn_msg = Twist()
        turn_msg.angular.z = 1.5708

        velocity_publisher.publish(turn_msg)

        time.sleep(2)

if __name__ == '__main__':

    try:
        move_turtle_square()
    except rospy.ROSInterruptException:
        pass
