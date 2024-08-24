#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import AprilTagDetectionArray, Twist2DStamped, FSMState
from sensor_msgs.msg import Range
from datetime import datetime, timedelta

class StopSignNode:
    def __init__(self):
        rospy.init_node('stop_sign_node')

        rospy.Subscriber('/duckiebot/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback)
        rospy.Subscriber('/duckiebot/front_center_tof_driver_node/range', Range, self.tof_callback)

        self.velocity_pub = rospy.Publisher('/duckiebot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.mode_pub = rospy.Publisher('/duckiebot/fsm_node/mode', FSMState, queue_size=1)

        self.stop_sign_ids = [24, 25, 26, 31, 32, 33]
        self.distance = 0.0
        self.stop_sign_detected = False
        self.last_detection_time = None
        self.ignore_duration = timedelta(seconds=3)  

        self.distance = None
        self.distance_threshold = 0.1
        self.obstacle_detected = False

    def tof_callback(self, msg):
        self.distance = float(msg.range)

    def enable_lane_controller(self):
        mode_msg = FSMState()
        mode_msg.state = "LANE_FOLLOWING"
        self.mode_pub.publish(mode_msg)
        rospy.loginfo("Duckiebot switched to autonomous")

    def disable_lane_controller(self):
        mode_msg = FSMState()
        mode_msg.state = "NORMAL_JOYSTICK_CONTROL"
        self.mode_pub.publish(mode_msg)
        rospy.loginfo("Duckiebot switched to manual")
    
    def stop_duckiebot(self, duration):
        rospy.loginfo("Duckiebot stopped")
        self.cmd_msg = Twist2DStamped()
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.velocity_pub.publish(self.cmd_msg)
        rospy.sleep(duration)

    def tag_callback(self, tag_array):
        if len(tag_array.detections) > 0:
            for detection in tag_array.detections:
                if detection.tag_id in self.stop_sign_ids:
                    self.distance = detection.transform.translation.z 
                    if self.distance < 0.2:
                        now = datetime.now()
                        if self.last_detection_time is None or (now - self.last_detection_time) > self.ignore_duration:
                            self.stop_sign_detected = True
                            self.last_detection_time = now
                    break
        elif len(tag_array.detections) == 0:
            if self.distance < self.distance_threshold:
                self.obstacle_detected = True
            else:
                self.obstacle_detected = False

    def main(self):
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            if self.stop_sign_detected:
                self.disable_lane_controller()
                self.stop_duckiebot(duration=3)
                self.enable_lane_controller()
                self.stop_sign_detected = False
            # if self.obstacle_detected == True:
            #     self.disable_lane_controller()
            # elif self.obstacle_detected == False:
            #     self.enable_lane_controller()
            rate.sleep()

if __name__ == '__main__':
    try:
        stop_sign_node = StopSignNode()
        stop_sign_node.main()
    except rospy.ROSInterruptException:
        pass
