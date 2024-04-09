#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from pid_controller import PIDController

class PIDNode:
    def __init__(self):
        rospy.init_node('pid_node')

        kp = 0.85
        ki = 0
        kd = 0.5

        self.controller = PIDController(kp, ki, kd)
        rospy.set_param("controller_ready", "ready")
        rospy.Subscriber('error', Float32, self.error_callback)
        self.control_pub = rospy.Publisher('control_input', Float32, queue_size=1)

    def error_callback(self, error_msg):
        error = error_msg.data
        dt = 0.1

        rospy.loginfo("Current error: {}".format(error))
        control_signal = self.controller.calculate(error, dt)
        rospy.loginfo("Control signal: {}".format(control_signal))
        self.control_pub.publish(Float32(control_signal))

if __name__ == '__main__':
    try:
        pid_node = PIDNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

