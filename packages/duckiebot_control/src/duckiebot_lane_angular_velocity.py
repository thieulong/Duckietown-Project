#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose
from pid_controller import PIDController

class LaneControllerNode:
    def __init__(self):
        rospy.init_node('lane_controller')

        # PID parameters for lateral control
        kp_lateral = 1.0
        ki_lateral = 0.01
        kd_lateral = 0.1

        # PID parameters for angular control
        kp_angular = 1.0
        ki_angular = 0.01
        kd_angular = 0.1

        # Initialize PID controllers
        self.pid_lateral = PIDController(kp_lateral, ki_lateral, kd_lateral)
        self.pid_angular = PIDController(kp_angular, ki_angular, kd_angular)

        # Subscriber for lane pose
        rospy.Subscriber('/duckiebot/lane_filter_node/lane_pose', LanePose, self.lane_pose_callback)

        # Publisher for control commands
        self.control_pub = rospy.Publisher('/duckiebot/lane_controller_node/car_cmd', Twist2DStamped, queue_size=1)

    def lane_pose_callback(self, lane_pose):
        # Lateral control
        lateral_error = lane_pose.d_ref - lane_pose.d
        dt_lateral = 0.1
        scaling_factor_lateral = 1
        control_signal_lateral = self.pid_lateral.calculate(lateral_error, dt_lateral)
        scaled_control_signal_lateral = control_signal_lateral * scaling_factor_lateral

        # Angular control
        angular_error = lane_pose.phi_ref - lane_pose.phi
        dt_angular = 0.1
        scaling_factor_angular = 1
        control_signal_angular = self.pid_angular.calculate(angular_error, dt_angular)
        scaled_control_signal_angular = control_signal_angular * scaling_factor_angular

        # Combine lateral and angular control signals
        final_control_signal = scaled_control_signal_lateral + scaled_control_signal_angular

        # Publish control commands
        car_cmd = Twist2DStamped()
        car_cmd.omega = final_control_signal
        self.control_pub.publish(car_cmd)

        # Print for debugging
        rospy.loginfo(f"Lateral Error: {lateral_error}, Angular Error: {angular_error}")

if __name__ == '__main__':
    try:
        lane_controller_node = LaneControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
