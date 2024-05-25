#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from pid_controller import PIDController
from duckietown_msgs.msg import AprilTagDetectionArray, Twist2DStamped
from geometry_msgs.msg import Twist

class FollowKeepDistanceNode:
    def __init__(self):
        rospy.init_node('follow_keep_distance')

        kp_angular = 2.5
        ki_angular = 0.01
        kd_angular = 0.1

        kp_linear = 1.5
        ki_linear = 0.01
        kd_linear = 0.4

        self.pid_angular = PIDController(kp_angular, ki_angular, kd_angular)
        self.pid_linear = PIDController(kp_linear, ki_linear, kd_linear)

        rospy.Subscriber('/duckiebot/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback)

        self.velocity_pub = rospy.Publisher('/duckiebot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)

    def tag_callback(self, tag_array):
        if len(tag_array.detections) > 0:
            tag = tag_array.detections[0]

            rotation = tag.transform.translation.x
            angular_error = rotation
            dt_angular = 0.1
            scaling_factor_angular = 4
            control_signal_angular = self.pid_angular.calculate(angular_error, dt_angular)
            scaled_control_signal_angular = control_signal_angular * scaling_factor_angular

            distance = tag.transform.translation.y
            linear_error = distance - 0.035
            dt_linear = 0.1
            scaling_factor_linear = 3
            control_signal_linear = self.pid_linear.calculate(linear_error, dt_linear)
            scaled_control_signal_linear = control_signal_linear * scaling_factor_linear

            car_cmd = Twist2DStamped()
            car_cmd.omega = scaled_control_signal_angular * -1.0
            car_cmd.v = scaled_control_signal_linear * -1.0
            self.velocity_pub.publish(car_cmd)

            rospy.loginfo(f"Angular Error: {angular_error}, Linear Error: {linear_error}")

        else:
            car_cmd = Twist2DStamped()
            car_cmd.omega = 0.0
            car_cmd.v = 0.0
            self.velocity_pub.publish(car_cmd)

if __name__ == '__main__':
    try:
        align_node = FollowKeepDistanceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


