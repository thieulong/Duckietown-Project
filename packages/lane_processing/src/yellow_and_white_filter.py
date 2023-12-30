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

    filename = sys.argv[1]

    rospy.init_node("color_filter", anonymous=True)
    pub_white = rospy.Publisher("/image_white", Image, queue_size=10)
    pub_yellow = rospy.Publisher("/image_yellow", Image, queue_size=10)
    pub_cropped = rospy.Publisher("/image_cropped", Image, queue_size=10)

    bridge = CvBridge()

    cv_img = cv2.imread(filename)

    height, width, _ = cv_img.shape

    cv_img = cv_img[height // 4 :, :]

    hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

    white_lower = np.array([50, 0, 200])
    white_upper = np.array([200, 100, 255])
    mask_white = cv2.inRange(hsv, white_lower, white_upper)

    yellow_lower = np.array([20, 0, 150])
    yellow_upper = np.array([40, 255, 255])
    mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)

    white_output = cv2.bitwise_and(cv_img, cv_img, mask=mask_white)

    yellow_output = cv2.bitwise_and(cv_img, cv_img, mask=mask_yellow)

    ros_img_cropped = bridge.cv2_to_imgmsg(cv_img, "bgr8")
    ros_img_white = bridge.cv2_to_imgmsg(white_output, "bgr8")
    ros_img_yellow = bridge.cv2_to_imgmsg(yellow_output, "bgr8")

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub_white.publish(ros_img_white)
        pub_yellow.publish(ros_img_yellow)
        pub_cropped.publish(ros_img_cropped)
        r.sleep()
