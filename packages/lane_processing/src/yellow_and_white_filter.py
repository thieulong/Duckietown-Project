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

    # initialize our node and create publishers for white and yellow images
    rospy.init_node("image_publisher", anonymous=True)
    pub_white = rospy.Publisher("/image_white", Image, queue_size=10)
    pub_yellow = rospy.Publisher("/image_yellow", Image, queue_size=10)
    pub_cropped = rospy.Publisher("/image_cropped", Image, queue_size=10)

    # we need to instantiate the class that does the CV-ROS conversion
    bridge = CvBridge()

    # read the image file into an OpenCV image
    cv_img = cv2.imread(filename)

    # get height and width of the image
    height, width, _ = cv_img.shape

    # Keep only the bottom half of the image
    cv_img = cv_img[height // 4 :, :]

    # convert to hsv for color filtering
    hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

    # set upper and lower boundaries for white color range
    white_lower = np.array([0, 0, 200])
    white_upper = np.array([180, 30, 255])
    mask_white = cv2.inRange(hsv, white_lower, white_upper)

    # set upper and lower boundaries for yellow color range
    yellow_lower = np.array([20, 100, 100])
    yellow_upper = np.array([30, 255, 255])
    mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)

    # filter only the white sections of the image
    white_output = cv2.bitwise_and(cv_img, cv_img, mask=mask_white)

    # filter only the yellow sections of the image
    yellow_output = cv2.bitwise_and(cv_img, cv_img, mask=mask_yellow)

    # convert to ROS sensor_msgs/Image
    ros_img_cropped = bridge.cv2_to_imgmsg(cv_img, "bgr8")
    ros_img_white = bridge.cv2_to_imgmsg(white_output, "bgr8")
    ros_img_yellow = bridge.cv2_to_imgmsg(yellow_output, "bgr8")

    # publish images to respective topics
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub_white.publish(ros_img_white)
        pub_yellow.publish(ros_img_yellow)
        pub_cropped.publish(ros_img_cropped)
        r.sleep()
