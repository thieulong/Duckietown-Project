#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import String
import random

class SignRecognition:
    def __init__(self):
            rospy.Subscriber('/duckiebot/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback)

            self.sign_pub = rospy.Publisher('duckiebot/road_sign_detected', String, queue_size=10)

            self.direction_pub = rospy.Publisher('duckiebot/intersection_direction', String, queue_size=10)

            self.current_sign_id = None
            rospy.Rate(1)

    def direction_randomize(self, directions):
        rospy.loginfo(f"Duckiebot is choosing direction ...")
        # directions = ["forward", "left", "right"]
        chosen_direction = random.choice(directions)
        rospy.loginfo(f"Chosen direction: {chosen_direction}")
        return chosen_direction

    def tag_callback(self, tag_array):

        self.detected_tag_id = None
        self.detected_sign = None
          
        self.tag_id_dict = {
            "Stop": [25, 31, 32, 33],
            "T-Intersection": 61,
            "Side Road Left": 65,
            "Side Road Right": 57
            }
        
        self.tag_id_info = {val: key for key, values in self.tag_id_dict.items() if isinstance(values, list) for val in values}

        self.tag_id_info.update({val: key for key, val in self.tag_id_dict.items() if not isinstance(val, list)})

        if len(tag_array.detections) > 0:
            tag = tag_array.detections[0]
            self.detected_tag_id = tag.tag_id

            if self.detected_tag_id != self.current_sign_id:
                if self.detected_tag_id in self.tag_id_info:
                    self.detected_sign = self.tag_id_info[self.detected_tag_id]

                    sign_msg = String()
                    sign_msg.data = self.detected_sign

                    self.sign_pub.publish(sign_msg)
                    self.current_sign_id = self.detected_tag_id

                    if self.detected_sign == "Stop":
                        direction = self.direction_randomize(directions=["forward", "left", "right"])
                    elif self.detected_sign == "T-Intersection":
                        direction = self.direction_randomize(directions=["left", "right"])
                    elif self.detected_sign == "Side Road Left":
                        direction = self.direction_randomize(directions=["forward", "left"])
                    elif self.detected_sign == "Side Road Right":
                        direction = self.direction_randomize(directions=["forward", "right"])

                    direction_msg = String()
                    direction_msg.data = direction
                    self.direction_pub.publish(direction_msg)
                    self.detected_sign = None
                                                             
                else:
                    rospy.logerr(f"Sign detected but unable to classify")
                    sign_msg = String()
                    sign_msg.data = "N/A"
                    self.sign_pub.publish(sign_msg)
                    direction_msg = String()
                    direction_msg.data = "N/A"
                    self.direction_pub.publish(direction_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('sign_recognition')
        sign_recogition = SignRecognition()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass