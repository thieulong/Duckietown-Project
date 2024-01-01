#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class DetectLane:
    def __init__(self):
        rospy.init_node('lane_detection', anonymous=True)

        self.bridge = CvBridge()

        rospy.Subscriber('/image_cropped', Image, self.cropped_image_callback)
        rospy.Subscriber('/image_white', Image, self.white_image_callback)
        rospy.Subscriber('/image_yellow', Image, self.yellow_image_callback)
        
        self.pub_edges_white = rospy.Publisher('/image_lines_white', Image, queue_size=10)
        self.pub_edges_yellow = rospy.Publisher('/image_lines_yellow', Image, queue_size=10)
        self.pub_edges = rospy.Publisher('/image_edges', Image, queue_size=10)
        self.pub_combined_lines = rospy.Publisher('/image_lines_all', Image, queue_size=10)

        self.edges = None 
        self.original_image = None
        self.line_detect_yellow = None
        self.line_detect_white = None

    def white_image_callback(self, msg):
        white_filtered_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_edge = cv2.cvtColor(self.edges, cv2.COLOR_GRAY2BGR)
        
        combined_image = cv2.bitwise_and(white_filtered_image, image_edge)
        combined_image = cv2.cvtColor(combined_image, cv2.COLOR_BGR2GRAY)

        # lines = cv2.HoughLinesP(combined_image, 1, np.pi / 180, threshold=200, minLineLength=50, maxLineGap=5)
        lines = cv2.HoughLinesP(combined_image, 1, np.pi / 180, threshold=10, minLineLength=10, maxLineGap=5)
        result_image = self.output_lines(self.original_image, lines)
        self.line_detect_white = result_image

        ros_img_edges = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')

        self.pub_edges_white.publish(ros_img_edges)
        self.publish_combined_lines(self.line_detect_white, self.line_detect_yellow)

    def yellow_image_callback(self, msg):
        yellow_filtered_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_edge = cv2.cvtColor(self.edges, cv2.COLOR_GRAY2BGR)

        combined_image = cv2.bitwise_and(yellow_filtered_image, image_edge)
        combined_image = cv2.cvtColor(combined_image, cv2.COLOR_BGR2GRAY)

        # lines = cv2.HoughLinesP(combined_image, 1, np.pi / 180, threshold=10, minLineLength=20, maxLineGap=5)
        lines = cv2.HoughLinesP(combined_image, 1, np.pi / 180, threshold=10, minLineLength=5, maxLineGap=5)
        result_image = self.output_lines(self.original_image, lines)
        self.line_detect_yellow = result_image

        ros_img_edges = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')

        self.pub_edges_yellow.publish(ros_img_edges)
        self.publish_combined_lines(self.line_detect_yellow, self.line_detect_white)

    def cropped_image_callback(self, msg):
        cropped_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.original_image = cropped_image

        blurred = cv2.GaussianBlur(cropped_image, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 200) 
        kernel = np.ones((5, 5), np.uint8)
        dilated_edges = cv2.dilate(edges, kernel, iterations=1)
        # self.edges = edges
        self.edges = dilated_edges

        ros_img_edges = self.bridge.cv2_to_imgmsg(dilated_edges, encoding='mono8')
        self.pub_edges.publish(ros_img_edges)

    def publish_combined_lines(self, white_lines, yellow_lines):
        if white_lines is None or yellow_lines is None:
            rospy.logwarn("One of the lines is None. Skipping publishing combined lines.")
            return

        common_size = (max(white_lines.shape[1], yellow_lines.shape[1]), max(white_lines.shape[0], yellow_lines.shape[0]))

        white_lines_resized = cv2.resize(white_lines, common_size)
        if len(white_lines_resized.shape) == 2:
            white_lines_bgr = cv2.cvtColor(white_lines_resized, cv2.COLOR_GRAY2BGR)
        else:
            white_lines_bgr = white_lines_resized

        yellow_lines_resized = cv2.resize(yellow_lines, common_size)
        if len(white_lines_resized.shape) == 2:
            yellow_lines_bgr = cv2.cvtColor(yellow_lines_resized, cv2.COLOR_GRAY2BGR)
        else:
            yellow_lines_bgr = yellow_lines_resized

        combined_lines = cv2.addWeighted(white_lines_bgr, 1, yellow_lines_bgr, 1, 0)

        ros_img_combined_lines = self.bridge.cv2_to_imgmsg(combined_lines, encoding='bgr8')
        self.pub_combined_lines.publish(ros_img_combined_lines)

    def output_lines(self, image, lines):
        output = np.copy(image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0], l[1]), (l[2], l[3]), (255, 0, 0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0], l[1]), 2, (0, 255, 0))
                cv2.circle(output, (l[2], l[3]), 2, (0, 0, 255))
        return output

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        lane_detector = DetectLane()
        lane_detector.run()
    except rospy.ROSInterruptException:
        pass
