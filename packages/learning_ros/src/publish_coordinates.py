#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Vector2D

def publish_coord():
    rospy.init_node('coord_publisher', anonymous=True)

    sensor_coord_pub = rospy.Publisher("/sensor_coord", Vector2D, queue_size=10)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        sensor_coord_msg = Vector2D(x=7, y=2)  

        sensor_coord_pub.publish(sensor_coord_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_coord()
    except rospy.ROSInterruptException:
        pass
