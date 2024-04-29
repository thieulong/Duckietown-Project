#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped

class DuckiebotMovement:
    def __init__(self):
        rospy.init_node('duckiebot_movement_custom', anonymous=True)
        
        rospy.Subscriber('/duckiebot/fsm_node/mode', FSMState, self.fsm_callback)
        rospy.Subscriber('/duckiebot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/duckiebot/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback)
        
        self.pub = rospy.Publisher('/duckiebot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
        self.cmd_msg = Twist2DStamped()

        self.left_encoder_list = list()
        self.right_encoder_list = list()

        self.current_state = None

        self.speed = 0.3
        self.distance = 500

    def fsm_callback(self, msg):
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.current_state = "MANUAL"
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":
            self.current_state = "AUTO"
            self.move_robot()

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
        rate = rospy.Rate(10)
        if not self.left_encoder_list or not self.right_encoder_list:
            rospy.logerr("Encoder lists are empty.")
            return

        start_left_encoder = self.left_encoder_list[-1]
        start_right_encoder = self.right_encoder_list[-1]
        forward_target_ticks = self.distance

        self.cmd_msg = Twist2DStamped()
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = self.speed
        self.cmd_msg.omega = 0.0 

        while not rospy.is_shutdown():
            current_left_encoder = self.left_encoder_list[-1]
            current_right_encoder = self.right_encoder_list[-1]
            print(current_left_encoder, current_right_encoder)

            if abs(current_left_encoder - start_left_encoder) >= forward_target_ticks or \
            abs(current_right_encoder - start_right_encoder) >= forward_target_ticks:
                rospy.loginfo("Forward target encoder ticks reached. Stopping robot.")
                break

            self.pub.publish(self.cmd_msg)
            rate.sleep()

        self.left_encoder_list.clear()
        self.right_encoder_list.clear()
        self.stop_robot()
        rospy.signal_shutdown("Completed movement task.")  

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        duckiebot_movement = DuckiebotMovement()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass


