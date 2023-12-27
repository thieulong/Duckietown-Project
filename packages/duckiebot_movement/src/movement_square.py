#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped

def move_robot():
    # Initialize the ROS node
    rospy.init_node('move_robot_node', anonymous=True)

    # Create a publisher for the /duckiebot/car_cmd_switch_node/cmd topic
    pub = rospy.Publisher('/duckiebot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)

    # Create a Twist2DStamped message
    cmd_msg = Twist2DStamped()
    cmd_msg.header.stamp = rospy.Time.now()
    cmd_msg.v = 0.0  # Set linear velocity (v) to 0.0
    cmd_msg.omega = 8.3  # Set angular velocity (omega) to 8.3

    # Publish the message
    pub.publish(cmd_msg)

    # Sleep for a short duration to allow the message to be published
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
