#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_srvs.srv import SetBool, SetBoolRequest
from duckietown_msgs.msg import AprilTagDetectionArray, Twist2DStamped, SegmentList, FSMState
from geometry_msgs.msg import Twist
import random

class IntersectionNavigation:
    def __init__(self):
        rospy.init_node('intersection_navigation')

        rospy.Subscriber('/duckiebot/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback)

        rospy.Subscriber('/duckiebot/line_detector_node/segment_list', SegmentList, self.segment_list_callback)

        rospy.Subscriber('/duckiebot/fsm_node/mode', FSMState, self.fsm_callback)

        self.velocity_pub = rospy.Publisher('/duckiebot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)

        self.stop_vehicle = False

        self.tag_id_dict = {
            "Stop": [31, 32, 33],
            "T-Intersection": 61,
            "Side Road Left": 65,
            "Side Road Right": 57
            }
        
        self.tag_id_info = {val: key for key, values in self.tag_id_dict.items() if isinstance(values, list) for val in values}
        self.tag_id_info.update({val: key for key, val in self.tag_id_dict.items() if not isinstance(val, list)})
        
        self.last_tag_id = None
        self.last_sign = None

        self.cmd_msg = Twist2DStamped()
    
    def intersection_randomize(self):
        rospy.loginfo(f"Duckiebot is at Intersection. Choosing direction randomly")
        directions = ["forward", "left", "right"]
        chosen_direction = random.choice(directions)
        rospy.loginfo(f"Chosen direction: {chosen_direction}")
        return chosen_direction
    
    def t_intersection_randomize(self):
        rospy.loginfo(f"Duckiebot is at T-Intersection. Choosing direction randomly")
        directions = ["left", "right"]
        chosen_direction = random.choice(directions)
        rospy.loginfo(f"Chosen direction: {chosen_direction}")
        return chosen_direction
    
    def sideroad_left_randomize(self):
        rospy.loginfo(f"Duckiebot is at Side Road Left. Choosing direction randomly")
        directions = ["forward", "left"]
        chosen_direction = random.choice(directions)
        rospy.loginfo(f"Chosen direction: {chosen_direction}")
        return chosen_direction
    
    def sideroad_right_randomize(self):
        rospy.loginfo(f"Duckiebot is at Side Road Right. Choosing direction randomly")
        directions = ["forward", "right"]
        chosen_direction = random.choice(directions)
        rospy.loginfo(f"Chosen direction: {chosen_direction}")
        return chosen_direction
    
    def handle_movement(self, direction):
        if direction == "forward":
            self.move_duckiebot_forward()
            rospy.sleep(1)
            self.stop_duckiebot()
            self.enable_lane_controller()
            self.stop_vehicle = False
        elif direction == "right":
            self.turn_duckiebot_right()
            rospy.sleep(2)
            self.stop_duckiebot()
            self.enable_lane_controller()
            self.stop_vehicle = False
        elif direction == "left":
            self.turn_duckiebot_left()
            rospy.sleep(2)
            self.stop_duckiebot()
            self.enable_lane_controller()
            self.stop_vehicle = False

    def fsm_callback(self, msg):
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_duckiebot()
        elif msg.state == "LANE_FOLLOWING":
            self.enter_autonomous()

    def enable_lane_controller(self):
        rospy.set_param('/lane_following/lane_controller', True)
        rospy.loginfo("Lane following enabled")

    def disable_lane_controller(self):
        rospy.set_param('/lane_following/lane_controller', False)
        rospy.loginfo("Lane following disabled")

    def stop_duckiebot(self):
        rospy.loginfo(f"Duckiebot stopped")
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.velocity_pub.publish(self.cmd_msg)

    def move_duckiebot_forward(self):
        rospy.loginfo(f"Duckiebot going forward")
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.41
        self.cmd_msg.omega = 0.0
        self.velocity_pub.publish(self.cmd_msg)

    def turn_duckiebot_right(self):
        rospy.loginfo(f"Duckiebot turning left")
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.41
        self.cmd_msg.omega = -8.3
        self.velocity_pub.publish(self.cmd_msg)

    def turn_duckiebot_left(self):
        rospy.loginfo(f"Duckiebot turning right")
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.41
        self.cmd_msg.omega = -8.3
        self.velocity_pub.publish(self.cmd_msg)

    def tag_callback(self, tag_array):
        if len(tag_array.detections) > 0:
            tag = tag_array.detections[0]
            self.last_tag_id = tag.tag_id
            if self.last_tag_id in self.tag_id_info:
                self.last_sign = self.tag_id_info[self.last_tag_id]
            else:
                rospy.logerr(f"Sign detected but unable to classify")

    def segment_list_callback(self, msg):
        for segment in msg.segments:
            color = segment.color
            pixels_normalized = segment.pixels_normalized

            if color == 2:
                for point in pixels_normalized:
                    x_normalized = point.x
                    y_normalized = point.y
                    rospy.loginfo(f"x: {x_normalized}, y: {y_normalized}")

                    if y_normalized > 0.6: self.stop_vehicle = True
                    else: self.stop_vehicle = False

    def enter_autonomous(self):
        rospy.loginfo("Duckiebot entered autonomous mode")
        rospy.loginfo(f"Stop Duckiebot: {self.stop_vehicle}")
        if self.stop_vehicle == True:
            self.disable_lane_controller()
            self.stop_duckiebot()
            rospy.loginfo(f"The last detected sign is: {self.last_sign}")
            rospy.sleep(5)
            if self.last_sign == "Stop":
                direction = self.intersection_randomize()
                self.handle_movement(direction=direction)
            elif self.last_sign == "T-Intersection":
                direction = self.t_intersection_randomize()
                self.handle_movement(direction=direction)
            elif self.last_sign == "Side Road Left":
                direction = self.sideroad_left_randomize()
                self.handle_movement(direction=direction)
            elif self.last_sign == "Side Road Right":
                direction = self.sideroad_right_randomize()
                self.handle_movement(direction=direction)
        else:
            self.enable_lane_controller()

if __name__ == '__main__':
    try:
        intersection_nav_node = IntersectionNavigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
