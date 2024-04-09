#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped

class TurnLeftNode:
    def __init__(self):
        rospy.init_node('left_turn_encoder')

        rospy.Subscriber('/duckiebot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback)
        self.velocity_pub = rospy.Publisher('/duckiebot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)

        self.encoder_list = list()

    def encoder_callback(self, msg):
        self.current_encoder = msg.data
        self.encoder_list.append(self.current_encoder)

    def run(self, direction):
        rate = rospy.Rate(10)  
        rate.sleep()
        if direction == 'left':
            self.max_encodings = 50
            target_encoder = self.encoder_list[0]
            while self.current_encoder in range(target_encoder - self.max_encodings, target_encoder + self.max_encodings):
                print(self.current_encoder)
                self.cmd_msg = Twist2DStamped()
                self.cmd_msg.header.stamp = rospy.Time.now()
                self.cmd_msg.v = 1
                self.cmd_msg.omega = 5  
                self.cmd_vel_pub.publish(self.cmd_msg)

                rate.sleep()

            self.cmd_msg  =Twist2DStamped()
            self.cmd_vel_pub.publish(self.cmd_msg)
            self.encoder_list = list()

if __name__ == '__main__':
    try:
        turn_left_node = TurnLeftNode()
        turn_left_node.run(direction='left')
    except rospy.ROSInterruptException:
        pass
