#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("ERROR incorrect number of arguments")
        print("Usage: %s <image filename>" % sys.argv[0])
        exit()

    # get the filename from the command line
    filename = sys.argv[1]

    # initialize our node and create a publisher as normal
    rospy.init_node("image_publisher", anonymous=True)
    pub = rospy.Publisher("image", Image, queue_size=10)

    # we need to instantiate the class that does the CV-ROS conversion
    bridge = CvBridge()

    # read the image file into an OpenCV image
    cv_img = cv2.imread(filename)

    # convert to hsv for color filtering
    hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

    # set upper and lower boundaries for white color range
    white_lower = np.array([0, 0, 200])
    white_upper = np.array([180, 30, 255])
    mask_white = cv2.inRange(hsv, white_lower, white_upper)

    # filter only the white sections of the image
    white_output = cv2.bitwise_and(cv_img, cv_img, mask=mask_white)

    # convert to a ROS sensor_msgs/Image
    ros_img = bridge.cv2_to_imgmsg(white_output, "bgr8")

    # publish ten times over a second
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(ros_img)
        r.sleep()
