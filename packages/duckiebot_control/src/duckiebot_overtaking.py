#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import FSMState
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

class OvertakeCommandNode:
    def __init__(self):
        rospy.init_node('overtake_command_node')

        rospy.Subscriber('/duckiebot/front_center_tof_driver_node/range', Range, self.tof_callback)
        rospy.Subscriber('/duckiebot/fsm_node/mode', FSMState, self.fsm_callback)

        self.mode_pub = rospy.Publisher('/duckiebot/fsm_node/mode', FSMState, queue_size=1)
        self.overtake_pub = rospy.Publisher('/duckiebot/overtaking/activated', Bool, queue_size=1)

        self.distance = 0.0
        self.distance_threshold = 0.15
        self.obstacle_detected_time = None
        self.obstacle_wait_duration = 5  
        self.overtake_started = False

        self.current_mode = "NORMAL_JOYSTICK_CONTROL"
        self.is_overtaking = False

    def tof_callback(self, msg):
        if not self.is_overtaking:  
            self.distance = float(msg.range)
            rospy.loginfo(f"Distance: {self.distance}")

    def fsm_callback(self, msg):
        if msg.state == "LANE_FOLLOWING" and not self.is_overtaking:
            self.current_mode = "LANE_FOLLOWING"
        elif msg.state == "NORMAL_JOYSTICK_CONTROL" and not self.is_overtaking:
            self.current_mode = "NORMAL_JOYSTICK_CONTROL"

    def enable_lane_controller(self):
        mode_msg = FSMState()
        mode_msg.state = "LANE_FOLLOWING"
        self.mode_pub.publish(mode_msg)
        rospy.loginfo("Duckiebot switched to autonomous")
        self.current_mode = "LANE_FOLLOWING"

    def disable_lane_controller(self):
        mode_msg = FSMState()
        mode_msg.state = "NORMAL_JOYSTICK_CONTROL"
        self.mode_pub.publish(mode_msg)
        rospy.loginfo("Duckiebot switched to manual")
        self.current_mode = "NORMAL_JOYSTICK_CONTROL"

    def main(self):
        rate = rospy.Rate(20)  
        while not rospy.is_shutdown():
            if not self.is_overtaking:  
                rospy.loginfo(f"Current mode: {self.current_mode}")
                if self.current_mode == "LANE_FOLLOWING":
                    if self.distance < self.distance_threshold:
                        rospy.loginfo("Detected obstacle in front")
                        if self.obstacle_detected_time is None:
                            self.disable_lane_controller()
                            rospy.loginfo("Start timer")
                            self.obstacle_detected_time = rospy.Time.now()
                            rospy.loginfo(f"Obstacle detect timer: {self.obstacle_detected_time.to_sec()}")
                        self.current_mode = "OBSTACLE_WAITING"
                elif self.current_mode == "OBSTACLE_WAITING":
                    if self.distance >= self.distance_threshold:
                        rospy.loginfo("Obstacle removed")
                        self.enable_lane_controller()
                        self.obstacle_detected_time = None
                        self.current_mode = "LANE_FOLLOWING"
                    elif self.obstacle_detected_time and (rospy.Time.now() - self.obstacle_detected_time).to_sec() > self.obstacle_wait_duration:
                        rospy.loginfo("Wait time exceeded! Proceed to overtaking.")
                        if not self.overtake_started:
                            self.overtake_started = True
                            rospy.loginfo("Start overtaking")
                            self.overtake_pub.publish(Bool(data=True)) 
                            rospy.sleep(5)
                            self.obstacle_detected_time = None
                            self.overtake_started = False
                        else: continue
                    else: continue
            rate.sleep()

if __name__ == '__main__':
    try:
        overtake_command_node = OvertakeCommandNode()
        overtake_command_node.main()
    except rospy.ROSInterruptException:
        pass
