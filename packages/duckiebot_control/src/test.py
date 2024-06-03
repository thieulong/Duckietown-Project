#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState
from sensor_msgs.msg import Range

class OvertakeNode:
    def __init__(self):
        rospy.init_node('overtake_node')

        rospy.Subscriber('/duckiebot/front_center_tof_driver_node/range', Range, self.tof_callback)
        rospy.Subscriber('/duckiebot/fsm_node/mode', FSMState, self.fsm_callback)

        self.velocity_pub = rospy.Publisher('/duckiebot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.mode_pub = rospy.Publisher('/duckiebot/fsm_node/mode', FSMState, queue_size=1)

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
    
    def stop_duckiebot(self, duration=0):
        rospy.loginfo("Duckiebot stopped")
        self.cmd_msg = Twist2DStamped()
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.velocity_pub.publish(self.cmd_msg)
        rospy.sleep(duration)

    def overtake_action_chain(self):
        self.is_overtaking = True
        rospy.loginfo("Duckiebot overtaking")
        
        rate = rospy.Rate(10)  

        end_time = rospy.Time.now() + rospy.Duration(0.8)
        while rospy.Time.now() < end_time:
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.0
            cmd_msg.omega = 3.0
            self.velocity_pub.publish(cmd_msg)
            # rate.sleep()

        end_time = rospy.Time.now() + rospy.Duration(1.2)
        while rospy.Time.now() < end_time:
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.2
            cmd_msg.omega = 0.0
            self.velocity_pub.publish(cmd_msg)
            # rate.sleep()

        end_time = rospy.Time.now() + rospy.Duration(0.8)
        while rospy.Time.now() < end_time:
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.0
            cmd_msg.omega = -3.0
            self.velocity_pub.publish(cmd_msg)
            # rate.sleep()

        end_time = rospy.Time.now() + rospy.Duration(1.8)
        while rospy.Time.now() < end_time:
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.2
            cmd_msg.omega = 0.0
            self.velocity_pub.publish(cmd_msg)
            # rate.sleep()

        end_time = rospy.Time.now() + rospy.Duration(0.5)
        while rospy.Time.now() < end_time:
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.0
            cmd_msg.omega = -3.0
            self.velocity_pub.publish(cmd_msg)
            # rate.sleep()

        end_time = rospy.Time.now() + rospy.Duration(1)
        while rospy.Time.now() < end_time:
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.2
            cmd_msg.omega = 0.0
            self.velocity_pub.publish(cmd_msg)
            # rate.sleep()
        
        self.stop_duckiebot()
        self.is_overtaking = False

    def main(self):
        rate = rospy.Rate(20)  
        while not rospy.is_shutdown():
            if not self.is_overtaking:  
                rospy.loginfo(f"Current mode: {self.current_mode}")
                if self.current_mode == "LANE_FOLLOWING":
                    if self.distance < self.distance_threshold:
                        self.disable_lane_controller()
                        rospy.loginfo("Detected obstacle in front")
                        if self.obstacle_detected_time is None:
                            rospy.loginfo("Start timer")
                            self.obstacle_detected_time = rospy.Time.now()
                            rospy.loginfo(f"Obstacle detect timer: {self.obstacle_detected_time.to_sec()}")
                        self.current_mode = "OBSTACLE_WAITING"
                        rate.sleep()
                elif self.current_mode == "OBSTACLE_WAITING":
                    if self.distance >= self.distance_threshold:
                        rospy.loginfo("Obstacle removed")
                        self.enable_lane_controller()
                        self.obstacle_detected_time = None
                        self.current_mode = "LANE_FOLLOWING"
                        rate.sleep()
                    elif (rospy.Time.now() - self.obstacle_detected_time).to_sec() > self.obstacle_wait_duration:
                        rospy.loginfo("Wait time exceeded! Proceed to overtaking.")
                        if not self.overtake_started:
                            self.overtake_started = True
                            rospy.loginfo("Start overtaking")
                            self.overtake_action_chain()
                            self.obstacle_detected_time = None
                            self.overtake_started = False
                            self.enable_lane_controller()
                            self.current_mode = "LANE_FOLLOWING"
                            rate.sleep()
            rate.sleep()

if __name__ == '__main__':
    try:
        overtake_node = OvertakeNode()
        overtake_node.main()
    except rospy.ROSInterruptException:
        pass
