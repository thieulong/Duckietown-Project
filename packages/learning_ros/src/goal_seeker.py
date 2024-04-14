#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32MultiArray
import math

class TurtleController:
    def __init__(self):
        rospy.init_node('goal_seeker', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.goal_subscriber = rospy.Subscriber('/goal_destination', Float32MultiArray, self.move_to_goal)
        
        self.pose = Pose()
        self.rate = rospy.Rate(10) 

    def update_pose(self, data):
        self.pose = data

    def move_to_goal(self, goal):
        goal_pose = Pose()
        goal_pose.x = goal.data[0]
        goal_pose.y = goal.data[1]

        goal_angle = math.atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
        while abs(goal_angle - self.pose.theta) > 0.01:
            velocity_msg = Twist()
            velocity_msg.angular.z = goal_angle - self.pose.theta
            self.velocity_publisher.publish(velocity_msg)
            self.rate.sleep()
        
        while math.sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)) > 0.1:
            velocity_msg.linear.x = math.sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
            velocity_msg.angular.z = 0
            self.velocity_publisher.publish(velocity_msg)
            self.rate.sleep()

        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0
        self.velocity_publisher.publish(velocity_msg)

if __name__ == '__main__':
    try:
        x = TurtleController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
