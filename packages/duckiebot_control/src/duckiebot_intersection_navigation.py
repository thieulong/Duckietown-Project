#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_srvs.srv import SetBool, SetBoolRequest
from duckietown_msgs.msg import AprilTagDetectionArray, Twist2DStamped
from geometry_msgs.msg import Twist

class IntersectionNavigation:
    def __init__(self):
        rospy.init_node('intersection_navigation')

        rospy.Subscriber('/duckiebot/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback)

        self.velocity_pub = rospy.Publisher('/duckiebot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)

        self.tag_id_dict = {}

    def tag_callback(self, tag_array):
        if len(tag_array.detections) > 0:
            tag = tag_array.detections[0]
            tag_id = tag.tag_id
            tag_distance = tag.transform.translation.y
            if tag_distance < 20:
                self.disable_lane_controller()
                self.stop_duckiebot(duration=5)

    def enable_lane_controller(self):
        rospy.set_param('/lane_following/lane_controller/switch', True)
        rospy.loginfo("Lane controller enabled")

    def disable_lane_controller(self):
        rospy.set_param('/lane_following/lane_controller/switch', False)
        rospy.loginfo("Lane controller disabled")

    def stop_duckiebot(self, duration):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.pub.publish(cmd_msg)
        rospy.loginfo(f"Duckiebot stopping for {duration} seconds.")