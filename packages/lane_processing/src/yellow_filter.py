#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

if __name__=="__main__":
    if len(sys.argv) < 1:
        print("ERROR incorrect number of arguments")
        print("Usage: %s <image filename>" % sys.argv[0])
        exit()
    
    # get the filename from the command line
    filename = sys.argv[1]
    
    # initialize our node and create a publisher as normal
    rospy.init_node("image_publisher", anonymous=True)
    pub = rospy.Publisher("image", Image, queue_size=10)
    
    # we need to instatiate the class that does the CV-ROS conversion
    bridge = CvBridge()
    
    #read the image file into an OpenCV image
    cv_img = cv2.imread(filename)
    
    # convert to hsv for colour filtering
    hsv = cv2.cvtColor(cv_img,cv2.COLOR_BGR2HSV)
    
    # set upper and lower boundaries for yellow colour range
    yellow_lower = np.array([20, 100, 100])
    yellow_upper = np.array([30, 255, 255])
    mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)
    
    # filter only the yellow sections of the image
    yellow_output = cv2.bitwise_and(cv_img, cv_img, mask=mask_yellow)
    
    # convert to a ROS sensor_msgs/Image
    ros_img = bridge.cv2_to_imgmsg(yellow_output, "bgr8")
    
    # publish ten times over a second
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        pub.publish(ros_img)
        r.sleep()
