#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose
from pid_controller import PIDController

class LaneControllerNode:
    def __init__(self):
        rospy.init_node('lane_control_node')

        kp_lateral = rospy.get_param("~kp_lateral", 2)
        ki_lateral = rospy.get_param("~ki_lateral", 1.5)
        kd_lateral = rospy.get_param("~kd_lateral", 0.01)

        kp_angular = rospy.get_param("~kp_angular", 2.25)
        ki_angular = rospy.get_param("~ki_angular", 1.75)
        kd_angular = rospy.get_param("~kd_angular", 0.01)

        self.pid_lateral = PIDController(kp_lateral, ki_lateral, kd_lateral)
        self.pid_angular = PIDController(kp_angular, ki_angular, kd_angular)

        rospy.Subscriber('/duckiebot/lane_filter_node/lane_pose', LanePose, self.lane_pose_callback)

        self.control_pub = rospy.Publisher('/duckiebot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)

    def lane_pose_callback(self, lane_pose):
        max_lateral_error = 3.0
        lateral_error = max(min(lane_pose.d_ref - lane_pose.d, max_lateral_error), -max_lateral_error)
        # rospy.loginfo(f"Lateral Error: {lateral_error}")
        if lateral_error < -0.02 or lateral_error > 0.02:
            dt_lateral = 0.05
            scaling_factor_lateral = -10
            control_signal_lateral = self.pid_lateral.calculate(lateral_error, dt_lateral)
            scaled_control_signal_lateral = control_signal_lateral * scaling_factor_lateral
        
            car_cmd = Twist2DStamped()
            car_cmd.v = 0
            car_cmd.omega = scaled_control_signal_lateral
            self.control_pub.publish(car_cmd)
        else:
            scaled_control_signal_lateral = 0

        rospy.loginfo(f"Lateral Control: {scaled_control_signal_lateral}")

        # max_angular_error = 3.0
        # angular_error = max(min(lane_pose.phi_ref - lane_pose.phi, max_angular_error), -max_angular_error)
        # # rospy.loginfo(f"Angular Error: {angular_error}")
        # if angular_error < -0.2 or angular_error > 0.2:
        #     dt_angular = 0.05
        #     scaling_factor_angular = 1.25
        #     control_signal_angular = self.pid_angular.calculate(angular_error, dt_angular)
        #     scaled_control_signal_angular = control_signal_angular * scaling_factor_angular
            
        #     car_cmd = Twist2DStamped()
        #     car_cmd.v = 0
        #     car_cmd.omega = scaled_control_signal_angular
        #     self.control_pub.publish(car_cmd)
        # else:
        #     scaled_control_signal_angular = 0

        # rospy.loginfo(f"Angular Control: {scaled_control_signal_angular}")

        # rospy.loginfo(f"Lateral Error: {lateral_error}, Angular Error: {angular_error}")

if __name__ == '__main__':
    try:
        lane_controller_node = LaneControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

