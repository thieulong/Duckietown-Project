#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import AprilTagDetectionArray, Twist2DStamped, BoolStamped, FSMState
import random
class IntersectionNavigation:
    def __init__(self):
        rospy.init_node('intersection_navigation')

        rospy.Subscriber('/duckiebot/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback)
        rospy.Subscriber('/duckiebot/stop_line_filter_node/at_stop_line', BoolStamped, self.stop_line_callback)
        rospy.Subscriber('/duckiebot/fsm_node/mode', FSMState, self.fsm_callback)

        self.mode_pub = rospy.Publisher('/duckiebot/fsm_node/mode', FSMState, queue_size=1)
        self.velocity_pub = rospy.Publisher('/duckiebot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)

        self.stop_line_detected = False
        self.movement_in_progress = False
        self.current_state = None

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
    
    def stop_duckiebot(self, duration):
        rospy.loginfo(f"Duckiebot stopped")
        self.cmd_msg = Twist2DStamped()
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0
        self.cmd_msg.omega = 0
        self.velocity_pub.publish(self.cmd_msg)
        rospy.sleep(duration)
    
    def move_duckiebot_forward(self, duration):
        self.disable_lane_controller()
        rospy.loginfo(f"Duckiebot going forward")
        self.cmd_msg = Twist2DStamped()
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.3
        self.cmd_msg.omega = 0.0
        self.velocity_pub.publish(self.cmd_msg)
        rospy.sleep(duration)
        self.stop_duckiebot()

    def turn_duckiebot_left(self, duration):
        self.disable_lane_controller()
        rospy.loginfo(f"Duckiebot turning left")
        self.cmd_msg = Twist2DStamped()
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.3
        self.cmd_msg.omega = 6
        self.velocity_pub.publish(self.cmd_msg)
        rospy.sleep(duration)
        self.stop_duckiebot()

    def turn_duckiebot_right(self, duration):
        self.disable_lane_controller()
        rospy.loginfo(f"Duckiebot turning right")
        self.cmd_msg = Twist2DStamped()
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.3
        self.cmd_msg.omega = -6
        self.velocity_pub.publish(self.cmd_msg)
        rospy.sleep(duration)
        self.stop_duckiebot()
    
    def handle_movement(self, direction):
        if direction == "forward":
            self.move_duckiebot_forward(duration=6)
        elif direction == "right":
            self.move_duckiebot_forward(duration=3)
            self.turn_duckiebot_right(duration=0.6)
            self.move_duckiebot_forward(duration=3)
        elif direction == "left":
            self.move_duckiebot_forward(duration=3)
            self.turn_duckiebot_left(duration=0.6)
            self.move_duckiebot_forward(duration=3)

    def fsm_callback(self, msg):
        if msg.state == "LANE_FOLLOWING":
            self.current_state = "AUTO"
        elif msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.current_state = "MANUAL"

    def enable_lane_controller(self):
        mode_msg = FSMState()
        mode_msg.state = "LANE_FOLLOWING"
        self.mode_pub.publish(mode_msg)
        # self.current_state = "AUTO"

    def disable_lane_controller(self):
        mode_msg = FSMState()
        mode_msg.state = "NORMAL_JOYSTICK_CONTROL"
        self.mode_pub.publish(mode_msg)
        # self.current_state = "MANUAL"

    def tag_callback(self, tag_array):
        if len(tag_array.detections) > 0:
            tag = tag_array.detections[0]
            self.last_tag_id = tag.tag_id
            if self.last_tag_id in self.tag_id_info:
                self.last_sign = self.tag_id_info[self.last_tag_id]
                rospy.loginfo(f"Last sign detected: {self.last_sign}")
            else:
                rospy.logerr(f"Sign detected but unable to classify")

    def stop_line_callback(self, msg):
        if msg.data == True:
            if self.current_state == "AUTO":
                if self.movement_in_progress == True:
                    self.stop_line_detected = False
                else:
                    self.stop_line_detected = True
                rospy.loginfo(f"Stop line detected: {self.stop_line_detected}")
            elif self.current_state == "MANUAL":
                # self.stop_line_detected = False
                rospy.loginfo("Stop line detected: SKIPPED")
            
    def enter_autonomous(self):
        rospy.loginfo(f"Duckiebot current state: {self.current_state}")
        if self.current_state == "AUTO":
            self.last_sign = "Stop"  
            rospy.loginfo(f"The last detected sign is: {self.last_sign}")
            if self.stop_line_detected == True:
                self.movement_in_progress = True
                rospy.loginfo("Stopping for 3 seconds")
                self.disable_lane_controller()
                self.stop_duckiebot(duration=3)
                if self.last_sign == "Stop":
                    rospy.loginfo(f"Duckiebot going forward")
                    self.cmd_msg = Twist2DStamped()
                    self.cmd_msg.header.stamp = rospy.Time.now()
                    self.cmd_msg.v = 0.3
                    self.cmd_msg.omega = 0.0
                    self.velocity_pub.publish(self.cmd_msg)
                    rospy.sleep(3)
                    self.stop_duckiebot(duration=0)
                self.movement_in_progress = False
                self.stop_line_detected = False
                self.enable_lane_controller()
                # elif self.last_sign == "T-Intersection":
                #     direction = self.t_intersection_randomize()
                #     self.handle_movement(direction=direction)
                # elif self.last_sign == "Side Road Left":
                #     direction = self.sideroad_left_randomize()
                #     self.handle_movement(direction=direction)
                # elif self.last_sign == "Side Road Right":
                #     direction = self.sideroad_right_randomize()
                #     self.handle_movement(direction=direction)
                # elif self.last_sign == None:
                #     self.stop_duckiebot()
                #     rospy.loginfo("No signs detected")
                #     self.stop_line_detected = False
            elif self.stop_line_detected == False and self.movement_in_progress == False:
                self.enable_lane_controller()
            elif self.stop_line_detected == False and self.movement_in_progress == True:
                rospy.loginfo("Duckiebot is moving")

        elif self.current_state == "MANUAL":
            self.disable_lane_controller()

if __name__ == '__main__':
    try:
        intersection_nav_node = IntersectionNavigation()
        intersection_nav_node.enter_autonomous()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
