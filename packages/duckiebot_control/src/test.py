#!/usr/bin/env python3

import rospy
import random
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray
from duckietown_msgs.msg import WheelEncoderStamped

class Autopilot:
    def __init__(self):
        
        rospy.init_node('autopilot_node', anonymous=True)

        rospy.on_shutdown(self.clean_shutdown)

        self.cmd_vel_pub = rospy.Publisher('/duckiebot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/duckiebot/fsm_node/mode', FSMState, queue_size=1)

        rospy.Subscriber('/duckiebot/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        rospy.Subscriber('/duckiebot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/duckiebot/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback)

        self.robot_state = "LANE_FOLLOWING"

        self.left_encoder_list = list()
        self.right_encoder_list = list()

        self.tag_id_dict = {
            "Stop": [24, 25, 26, 31, 32, 33],
            "T-Intersection": [65, 11],
            "Side Road Left": [10],
            "Side Road Right": [57]
            }
        
        self.tag_id_info = {val: key for key, values in self.tag_id_dict.items() if isinstance(values, list) for val in values}
        self.tag_id_info.update({val: key for key, val in self.tag_id_dict.items() if not isinstance(val, list)})

        self.tag_id = None
        self.tag_info = None
        self.tag_distance = None
        self.detect_tag = False
        self.tag_threshold_distance = 0.3

        self.is_turning = False

        rospy.spin() 

    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING":
            return
        
        self.move_robot(msg.detections)

    def left_encoder_callback(self, msg):
        self.left_current_encoder = msg.data
        self.left_encoder_list.append(self.left_current_encoder)

    def right_encoder_callback(self, msg):
        self.right_current_encoder = msg.data
        self.right_encoder_list.append(self.right_current_encoder)
 
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def set_state(self, state):
        rospy.loginfo(f"Navigating in progress: {self.is_turning}")
        self.robot_state = state
        if self.is_turning:
            return
        else:
            state_msg = FSMState()
            state_msg.header.stamp = rospy.Time.now()
            state_msg.state = self.robot_state
            self.state_pub.publish(state_msg)

    def handle_movement(self, direction):
        rate = rospy.Rate(10)
        self.is_turning = True
        if direction == "forward":
            rospy.loginfo(f"Duckiebot moving forward")
            self.max_encodings = 500
            self.target_encoder = self.left_encoder_list[-1]
            while self.left_current_encoder in range(self.target_encoder-self.max_encodings, self.target_encoder+self.max_encodings):
                self.cmd_msg = Twist2DStamped()
                self.cmd_msg.header.stamp = rospy.Time.now()
                self.cmd_msg.v = 0.2
                self.cmd_msg.omega = 0.0
                self.cmd_vel_pub.publish(self.cmd_msg)
                rate.sleep()
            
            self.stop_robot()
            
        elif direction == "right":
            rospy.loginfo(f"Duckiebot turning right")
            self.max_encodings = 200
            self.target_encoder = self.right_encoder_list[-1]
            while self.right_current_encoder in range(self.target_encoder-self.max_encodings, self.target_encoder+self.max_encodings):
                self.cmd_msg = Twist2DStamped()
                self.cmd_msg.header.stamp = rospy.Time.now()
                self.cmd_msg.v = 0.2
                self.cmd_msg.omega = -2.5
                self.cmd_vel_pub.publish(self.cmd_msg)
                rate.sleep()
            
            self.stop_robot()

        elif direction == "left":
            rospy.loginfo(f"Duckiebot turning left")
            self.max_encodings = 450
            self.target_encoder = self.left_encoder_list[-1]
            while self.left_current_encoder in range(self.target_encoder-self.max_encodings, self.target_encoder+self.max_encodings):
                self.cmd_msg = Twist2DStamped()
                self.cmd_msg.header.stamp = rospy.Time.now()
                self.cmd_msg.v = 0.2
                self.cmd_msg.omega = 1.5
                self.cmd_vel_pub.publish(self.cmd_msg)
                rate.sleep()
            
            self.stop_robot()

        self.is_turning = False

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

    def move_robot(self, detections):

        if len(detections) == 0:
            return

        if len(detections) > 0:
            direction = None
            self.detect_tag = True
            tag = detections[0]
            self.tag_id = tag.tag_id
            self.tag_distance = tag.transform.translation.z
            if self.tag_id in self.tag_id_info:
                self.tag_info = self.tag_id_info[self.tag_id]
            else: 
                rospy.loginfo("Trouble recognizing this sign")

            rospy.loginfo(f"Tag ID: {self.tag_id}")
            rospy.loginfo(f"Tag Distance: {self.tag_distance}")
            rospy.loginfo(f"Tag Info: {self.tag_info}")

            if self.tag_distance <= self.tag_threshold_distance:
                self.set_state("NORMAL_JOYSTICK_CONTROL")

                self.cmd_msg = Twist2DStamped()
                self.cmd_msg.header.stamp = rospy.Time.now()
                self.cmd_msg.v = 0.05
                self.cmd_msg.omega = 0.0
                self.cmd_vel_pub.publish(self.cmd_msg)
                rospy.sleep(1.5)

                self.stop_robot()
                rospy.sleep(3)

                if self.tag_info == "Stop":
                    direction = self.intersection_randomize()
                elif self.tag_info == "T-Intersection":
                    direction = self.t_intersection_randomize()
                elif self.tag_info == "Side Road Left":
                    direction = self.sideroad_left_randomize()
                elif self.tag_info == "Side Road Right":
                    direction = self.sideroad_right_randomize()
                else:
                    direction = "forward"
                    rospy.logwarn("Unable to read last sign, moving forward")
                rospy.loginfo(f"Duckiebot direction: {direction}")

            self.handle_movement(direction=direction) 
            self.set_state("LANE_FOLLOWING")


if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
