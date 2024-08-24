#!/usr/bin/env python3

import rospy 
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from duckietown_msgs.msg import Segment, SegmentList, Vector2D
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool, SetBoolResponse
from cv_bridge import CvBridge
from copy import deepcopy

class CustomLaneDetection:
    def __init__(self):
        rospy.init_node('custom_lane_detection', anonymous=True)

        self.bridge = CvBridge()

        rospy.Service('line_detector_node/switch', SetBool, self.ld_switch)
        rospy.Service('lane_filter_node/switch', SetBool, self.lf_switch)

        rospy.Subscriber("/duckiebot/camera_node/image/compressed", CompressedImage, self.lane_processing_callback, queue_size=1, buff_size=2**24)

        self.pub_edges_white = rospy.Publisher('/image_lines_white', Image, queue_size=10)
        self.pub_edges_yellow = rospy.Publisher('/image_lines_yellow', Image, queue_size=10)
        self.pub_edges = rospy.Publisher('/image_edges', Image, queue_size=10)
        self.pub_combined_results = rospy.Publisher('/image_lines_all', Image, queue_size=10)
        self.pub_original_combined_results = rospy.Publisher('/original_image_lines_all', Image, queue_size=10)
        self.pub_segment_list = rospy.Publisher('/duckiebot/line_detector_node/segment_list', SegmentList, queue_size=10)

    def lane_processing_callback(self, msg):
        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        original_image = deepcopy(image)

        image_size = (160, 120)
        offset = 40
        new_image = cv2.resize(image, image_size, interpolation=cv2.INTER_NEAREST)
        image = new_image[offset:, :]

        height, width, _ = image.shape
        image_cropped = image[height // 100 :, :]

        arr_cutoff = np.array([0, offset, 0, offset])
        arr_ratio = np.array([1. / image_size[0], 1. / image_size[1], 1. / image_size[0], 1. / image_size[1]])

        image_cropped_blurred = cv2.GaussianBlur(image_cropped, (5, 5), 0)
        edges = cv2.Canny(image_cropped_blurred, 50, 200) 
        kernel = np.ones((5, 5), np.uint8)
        dilated_edges = cv2.dilate(edges, kernel, iterations=1)
        dilated_edges = cv2.cvtColor(dilated_edges, cv2.COLOR_GRAY2BGR)

        ros_dilated_edges = self.bridge.cv2_to_imgmsg(dilated_edges, encoding='bgr8')
        self.pub_edges.publish(ros_dilated_edges)

        hsv = cv2.cvtColor(image_cropped, cv2.COLOR_BGR2HSV)

        white_lower = np.array([50, 0, 200])
        white_upper = np.array([200, 100, 255])
        mask_white = cv2.inRange(hsv, white_lower, white_upper)

        white_filtered = cv2.bitwise_and(image_cropped, image_cropped, mask=mask_white)
        white_edges = cv2.bitwise_and(white_filtered, dilated_edges)
        white_edges = cv2.cvtColor(white_edges, cv2.COLOR_BGR2GRAY)
        white_lines = cv2.HoughLinesP(white_edges, 1, np.pi / 180, threshold=10, minLineLength=10, maxLineGap=5)
        if white_lines is not None:
            white_lines_normalized = (white_lines + arr_cutoff) * arr_ratio

        white_segment_list = SegmentList()
        for line in white_lines_normalized:
            segment = Segment()
            segment.color = segment.WHITE
            segment.pixels_normalized = [Vector2D(x=line[0][0], y=line[0][1]),
                                         Vector2D(x=line[0][2], y=line[0][3])]
            white_segment_list.segments.append(segment)
        self.pub_segment_list.publish(white_segment_list)

        white_output = self.output_lines_green(image_cropped, white_lines)

        ros_white_results = self.bridge.cv2_to_imgmsg(white_output, encoding='bgr8')
        self.pub_edges_white.publish(ros_white_results)

        yellow_lower = np.array([20, 0, 150])
        yellow_upper = np.array([40, 255, 255])
        mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)

        yellow_filtered = cv2.bitwise_and(image_cropped, image_cropped, mask=mask_yellow)
        yellow_edges = cv2.bitwise_and(yellow_filtered, dilated_edges)
        yellow_edges = cv2.cvtColor(yellow_edges, cv2.COLOR_BGR2GRAY)
        yellow_lines = cv2.HoughLinesP(yellow_edges, 1, np.pi / 180, threshold=10, minLineLength=5, maxLineGap=5)
        if yellow_lines is not None:
            yellow_lines_normalized = (yellow_lines + arr_cutoff) * arr_ratio

        yellow_segment_list = SegmentList()
        for line in yellow_lines_normalized:
            segment = Segment()
            segment.color = segment.YELLOW
            segment.pixels_normalized = [Vector2D(x=line[0][0], y=line[0][1]),
                                         Vector2D(x=line[0][2], y=line[0][3])]
            yellow_segment_list.segments.append(segment)
        self.pub_segment_list.publish(yellow_segment_list)

        yellow_output = self.output_lines_blue(image_cropped, yellow_lines)

        ros_yellow_results = self.bridge.cv2_to_imgmsg(yellow_output, encoding='bgr8')
        self.pub_edges_yellow.publish(ros_yellow_results)

        combined_lines = cv2.addWeighted(white_output, 1, yellow_output, 1, 0)
        ros_combined_results = self.bridge.cv2_to_imgmsg(combined_lines, encoding='bgr8')
        self.pub_combined_results.publish(ros_combined_results)

        def revert_coordinates(line):
            x1, y1, x2, y2 = line
            x1_orig = int(x1 * (original_image.shape[1] / image_size[0]))
            y1_orig = int((y1 + offset) * (original_image.shape[0] / image_size[1]))
            x2_orig = int(x2 * (original_image.shape[1] / image_size[0]))
            y2_orig = int((y2 + offset) * (original_image.shape[0] / image_size[1]))
            return x1_orig, y1_orig, x2_orig, y2_orig

        def draw_lines_on_original(lines, color):
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = revert_coordinates(line[0])
                    cv2.line(original_image, (x1, y1), (x2, y2), color, 4, cv2.LINE_AA) 

        draw_lines_on_original(white_lines, (0, 255, 0))  
        draw_lines_on_original(yellow_lines, (255, 0, 0))  

        ros_original_combined_results = self.bridge.cv2_to_imgmsg(original_image, encoding='bgr8')
        self.pub_original_combined_results.publish(ros_original_combined_results)

    def output_lines_blue(self, image, lines):
        output = np.copy(image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0], l[1]), (l[2], l[3]), (255, 0, 0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0], l[1]), 2, (0, 255, 0))
                cv2.circle(output, (l[2], l[3]), 2, (0, 0, 255))
        return output
    
    def output_lines_green(self, image, lines):
        output = np.copy(image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0], l[1]), (l[2], l[3]), (0, 255, 0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0], l[1]), 2, (0, 255, 0))
                cv2.circle(output, (l[2], l[3]), 2, (0, 0, 255))
        return output

    def ld_switch(self, msg):
        return True, ""

    def lf_switch(self, msg):
        return True, ""

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        lane_detect = CustomLaneDetection()
        lane_detect.run()
    except rospy.ROSInterruptException:
        pass
