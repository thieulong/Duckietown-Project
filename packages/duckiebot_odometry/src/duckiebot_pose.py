#!/usr/bin/env python3

from math import sin, cos
import rospy 
from odometry_hw.msg import DistWheel, Pose2D

class UpdatePose:
    def __init__(self):
        rospy.set_param("/odom_ready", "ready")
        rospy.Subscriber("/dist_wheel", DistWheel, self.calculate_pose)
        self.robot_pose_pub = rospy.Publisher("/pose", Pose2D, queue_size=10)

        self.L = 0.05
        self.x = 0
        self.y = 0
        self.theta = 0

    def calculate_pose(self, distwheel_msg):
        if distwheel_msg.dist_wheel_left is None or distwheel_msg.dist_wheel_right is None:
            d_left = 0
            d_right = 0
        else:
            d_left = distwheel_msg.dist_wheel_left
            d_right = distwheel_msg.dist_wheel_right

        delta_s = (d_left + d_right) / 2
        delta_theta = (d_right - d_left) / (self.L * 2)

        delta_x = delta_s * cos(self.theta + (delta_theta / 2))
        delta_y = delta_s * sin(self.theta + (delta_theta / 2))

        x_new = self.x + delta_x
        y_new = self.y + delta_y
        theta_new = self.theta + delta_theta

        robot_pose = Pose2D()
        robot_pose.x = x_new
        robot_pose.y = y_new
        robot_pose.theta = theta_new
        self.robot_pose_pub.publish(robot_pose)

        self.x = x_new
        self.y = y_new
        self.theta = theta_new

if __name__ == '__main__':
    rospy.init_node('update_pose', anonymous=True)
    update_pose = UpdatePose()
    rospy.spin()
