#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import AprilTagDetectionArray, Twist2DStamped, BoolStamped, FSMState, WheelEncoderStamped
import random
class IntersectionNavigation:
    def __init__(self):
        rospy.Subscriber('/duckiebot/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback)
        rospy.Subscriber('/duckiebot/stop_line_filter_node/at_stop_line', BoolStamped, self.stop_line_callback)
        rospy.Subscriber('/duckiebot/fsm_node/mode', FSMState, self.fsm_callback)
        rospy.Subscriber('/duckiebot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/duckiebot/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback)

        self.mode_pub = rospy.Publisher('/duckiebot/fsm_node/mode', FSMState, queue_size=10)
        self.velocity_pub = rospy.Publisher('/duckiebot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)

        self.stop_line_detected = False
        self.movement_in_progress = False
        self.current_state = None

        # self.tag_id_dict = {
        #     "Stop": [25, 26, 31, 32, 33],
        #     "T-Intersection": [61, 10],
        #     "Side Road Left": [65, 11],
        #     "Side Road Right": 57
        #     }

        self.tag_id_dict = {
            "Stop": [24, 25, 26, 31, 32, 33],
            "T-Intersection": [65, 11],
            "Side Road Left": [10],
            "Side Road Right": [57]
            }
        
        self.tag_id_info = {val: key for key, values in self.tag_id_dict.items() if isinstance(values, list) for val in values}
        self.tag_id_info.update({val: key for key, val in self.tag_id_dict.items() if not isinstance(val, list)})
        
        self.last_tag_id = None
        self.last_sign = None

        self.left_encoder_list = list()
        self.right_encoder_list = list()
    
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
    
    def left_encoder_callback(self, msg):
        self.left_current_encoder = msg.data
        self.left_encoder_list.append(self.left_current_encoder)

    def right_encoder_callback(self, msg):
        self.right_current_encoder = msg.data
        self.right_encoder_list.append(self.right_current_encoder)
    
    def stop_duckiebot(self, duration):
        rospy.loginfo(f"Duckiebot stopped")
        self.cmd_msg = Twist2DStamped()
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.velocity_pub.publish(self.cmd_msg)
        rospy.sleep(duration)
    
    def handle_movement(self, direction):
        rate = rospy.Rate(10)
        rate.sleep()
        self.disable_lane_controller()
        if direction == "forward":
            rospy.loginfo(f"Duckiebot moving forward")
            self.max_encodings = 500
            self.target_encoder = self.left_encoder_list[0]
            while self.left_current_encoder in range(self.target_encoder-self.max_encodings, self.target_encoder+self.max_encodings):
                self.cmd_msg = Twist2DStamped()
                self.cmd_msg.header.stamp = rospy.Time.now()
                self.cmd_msg.v = 0.3
                self.cmd_msg.omega = 0.0
                self.velocity_pub.publish(self.cmd_msg)
                rate.sleep()
            
            self.stop_duckiebot(duration=0)
            self.encoder_list = list()
            
        elif direction == "right":
            rospy.loginfo(f"Duckiebot turning right")
            self.max_encodings = 300
            self.target_encoder = self.right_encoder_list[0]
            while self.right_current_encoder in range(self.target_encoder-self.max_encodings, self.target_encoder+self.max_encodings):
                self.cmd_msg = Twist2DStamped()
                self.cmd_msg.header.stamp = rospy.Time.now()
                self.cmd_msg.v = 0.3
                self.cmd_msg.omega = -2.0
                self.velocity_pub.publish(self.cmd_msg)
                rate.sleep()
            
            self.stop_duckiebot(duration=0)
            self.encoder_list = list()

        elif direction == "left":
            rospy.loginfo(f"Duckiebot turning left")
            self.max_encodings = 500
            self.target_encoder = self.left_encoder_list[0]
            while self.left_current_encoder in range(self.target_encoder-self.max_encodings, self.target_encoder+self.max_encodings):
                self.cmd_msg = Twist2DStamped()
                self.cmd_msg.header.stamp = rospy.Time.now()
                self.cmd_msg.v = 0.3
                self.cmd_msg.omega = 1.0
                self.velocity_pub.publish(self.cmd_msg)
                rate.sleep()
            
            self.stop_duckiebot(duration=0)
            self.encoder_list = list()

    def fsm_callback(self, msg):
        if msg.state == "LANE_FOLLOWING":
            self.current_state = "AUTONOMOUS"
        elif msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.current_state = "MANUAL"

    def enable_lane_controller(self):
        mode_msg = FSMState()
        mode_msg.state = "LANE_FOLLOWING"
        self.mode_pub.publish(mode_msg)
        rospy.loginfo("Duckiebot switched to autonomous")
        self.current_state = "AUTONOMOUS"

    def disable_lane_controller(self):
        mode_msg = FSMState()
        mode_msg.state = "NORMAL_JOYSTICK_CONTROL"
        self.mode_pub.publish(mode_msg)
        rospy.loginfo("Duckiebot switched to manual")
        self.current_state = "MANUAL"

    def tag_callback(self, tag_array):
        if len(tag_array.detections) > 0:
            tag = tag_array.detections[0]
            self.last_tag_id = tag.tag_id
            if self.last_tag_id in self.tag_id_info:
                self.last_sign = self.tag_id_info[self.last_tag_id]
                # rospy.loginfo(f"Last sign detected: {self.last_sign}")
            # else:
                # rospy.logerr(f"Sign detected but unable to classify")
 
    def stop_line_callback(self, msg):
        self.stop_line_detected = msg.data

    def main_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo(f"Duckiebot state: {self.current_state}")
            rospy.loginfo(f"Stop lane detected: {self.stop_line_detected}")
            # rospy.loginfo(f"Movement in progress: {self.movement_in_progress}")
            rospy.loginfo(f"Last sign detected: {self.last_sign}")
            rospy.loginfo(f"Last sign ID: {self.last_tag_id}")

            if self.current_state == "AUTONOMOUS":
                if self.stop_line_detected == True:
                    if self.movement_in_progress == False:

                        self.movement_in_progress = True
                        rospy.loginfo("Stopping Duckiebot")
                        self.disable_lane_controller()
                        self.stop_duckiebot(duration=1)

                        rate.sleep()

                        if self.last_sign == "Stop":
                            direction = self.intersection_randomize()
                        elif self.last_sign == "T-Intersection":
                            direction = self.t_intersection_randomize()
                        elif self.last_sign == "Side Road Left":
                            direction = self.sideroad_left_randomize()
                        elif self.last_sign == "Side Road Right":
                            direction = self.sideroad_right_randomize()
                        else:
                            direction = "forward"
                            rospy.logwarn("Unable to read last sign, moving forward")
                        rospy.loginfo(f"Duckiebot direction: {direction}")

                        self.handle_movement(direction=direction)
                        rate.sleep()
                        self.movement_in_progress = False
                        self.last_sign = None
                        self.last_tag_id = None
                        self.enable_lane_controller()

                        rate.sleep()

                    elif self.movement_in_progress == True:
                        rospy.loginfo("Movement in progress, skipping stop_line_callback")
                        return
                    
                elif self.stop_line_detected == False:
                    rate.sleep()
                    self.enable_lane_controller()
            elif self.current_state == "MANUAL":
                rate.sleep()
                self.disable_lane_controller()

            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('intersection_navigation')
        intersection_nav_node = IntersectionNavigation()
        intersection_nav_node.main_loop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
