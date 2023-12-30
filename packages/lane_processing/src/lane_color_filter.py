#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ColorFilterNode:
    def __init__(self):
        rospy.init_node("color_filter", anonymous=True)

        self.pub_white = rospy.Publisher("/image_white", Image, queue_size=10)
        self.pub_yellow = rospy.Publisher("/image_yellow", Image, queue_size=10)
        self.pub_cropped = rospy.Publisher("/image_cropped", Image, queue_size=10)

        self.bridge = CvBridge()

        # Subscribe to the /image topic
        rospy.Subscriber("/image", Image, self.image_callback)

    def image_callback(self, msg):
        # Convert the ROS Image message to OpenCV image
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process the image
        self.process_image(cv_img)

    def process_image(self, cv_img):
        height, width, _ = cv_img.shape

        # Keep only the bottom half of the image
        cv_img_cropped = cv_img[height // 2 :, :]

        hsv = cv2.cvtColor(cv_img_cropped, cv2.COLOR_BGR2HSV)

        # white_lower = np.array([0, 0, 200])
        # white_upper = np.array([180, 30, 255])
        white_lower = np.array([25, 0, 120])
        white_upper = np.array([120, 50, 255])
        mask_white = cv2.inRange(hsv, white_lower, white_upper)

        # yellow_lower = np.array([20, 100, 100])
        # yellow_upper = np.array([30, 255, 255])
        yellow_lower = np.array([0, 150, 220])
        yellow_upper = np.array([120, 255, 255])
        mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)

        white_output = cv2.bitwise_and(cv_img_cropped, cv_img_cropped, mask=mask_white)
        yellow_output = cv2.bitwise_and(cv_img_cropped, cv_img_cropped, mask=mask_yellow)

        # Convert processed images to ROS Image messages
        ros_img_cropped = self.bridge.cv2_to_imgmsg(cv_img_cropped, "bgr8")
        ros_img_white = self.bridge.cv2_to_imgmsg(white_output, "bgr8")
        ros_img_yellow = self.bridge.cv2_to_imgmsg(yellow_output, "bgr8")

        # Publish the processed images
        self.pub_white.publish(ros_img_white)
        self.pub_yellow.publish(ros_img_yellow)
        self.pub_cropped.publish(ros_img_cropped)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        color_filter_node = ColorFilterNode()
        color_filter_node.run()
    except rospy.ROSInterruptException:
        pass
