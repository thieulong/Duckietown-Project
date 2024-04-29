#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped
from sensor_msgs.msg import Range

class DuckiebotMovement:
    def __init__(self):
        rospy.init_node('duckiebot_movement_custom', anonymous=True)
        
        rospy.Subscriber('/duckiebot/fsm_node/mode', FSMState, self.fsm_callback)
        rospy.Subscriber('/duckiebot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/duckiebot/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback)
        rospy.Subscriber('/duckiebot/front_center_tof_driver_node/range', Range, self.tof_callback)
        
        self.pub = rospy.Publisher('/duckiebot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
        self.cmd_msg = Twist2DStamped()

        self.left_encoder_list = list()
        self.right_encoder_list = list()

        self.current_state = None
        self.tof_range = 0

    def fsm_callback(self, msg):
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.current_state = "MANUAL"
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":
            self.current_state = "AUTO"
            self.move_robot()

    def tof_callback(self, msg):
        self.tof_range = float(msg.range)


    def left_encoder_callback(self, msg):
        self.left_current_encoder = msg.data
        self.left_encoder_list.append(self.left_current_encoder)

    def right_encoder_callback(self, msg):
        self.right_current_encoder = msg.data
        self.right_encoder_list.append(self.right_current_encoder)

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def move_robot(self):
        rospy.loginfo(f"Current state: {self.current_state}")
        for i in range(4):
            if self.current_state == "AUTO":
                rate = rospy.Rate(10) 
                if not self.left_encoder_list or not self.right_encoder_list:
                    rospy.logerr("Encoder lists are empty.")
                    return

                start_left_encoder = self.left_encoder_list[-1]
                start_right_encoder = self.right_encoder_list[-1]
                forward_target_ticks = 500

                self.cmd_msg = Twist2DStamped()

                while not rospy.is_shutdown():
                    current_left_encoder = self.left_encoder_list[-1]
                    current_right_encoder = self.right_encoder_list[-1]

                    if abs(current_left_encoder - start_left_encoder) >= forward_target_ticks or \
                    abs(current_right_encoder - start_right_encoder) >= forward_target_ticks:
                        rospy.loginfo("Forward target encoder ticks reached. Stopping robot.")
                        break

                    if self.tof_range < 0.1:
                        rospy.loginfo("Obstacle ahead, stopping Duckiebot")
                        rospy.loginfo(f"Distance ahead: {self.tof_range}")
                        self.cmd_msg.header.stamp = rospy.Time.now()
                        self.cmd_msg.v = 0.0
                        self.cmd_msg.omega = 0.0 
                    else:
                        self.cmd_msg.header.stamp = rospy.Time.now()
                        self.cmd_msg.v = 0.3  
                        self.cmd_msg.omega = 0.0 
                    
                    self.pub.publish(self.cmd_msg)
                    rate.sleep()

                self.stop_robot()
            
            else: self.stop_robot()

            start_left_encoder = self.left_encoder_list[-1]
            start_right_encoder = self.right_encoder_list[-1]

            right_target_ticks = 55
            # left_target_ticks = 0

            self.cmd_msg = Twist2DStamped()
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = 5.0

            while not rospy.is_shutdown():
                # current_left_encoder = self.left_encoder_list[-1]
                current_right_encoder = self.right_encoder_list[-1]
                print(current_left_encoder, current_right_encoder)

                if abs(current_right_encoder - start_right_encoder) >= right_target_ticks:
                    rospy.loginfo("Turn right target encoder ticks reached. Stopping robot.")
                    break

                self.pub.publish(self.cmd_msg)
                rate.sleep()

            self.stop_robot()

        self.left_encoder_list.clear()
        self.right_encoder_list.clear()

        self.stop_robot()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        duckiebot_movement = DuckiebotMovement()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass


