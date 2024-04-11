#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class TurtleController:
    def __init__(self):
        rospy.init_node('turtlesim_goal', anonymous=True)
        
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        rospy.Subscriber('turtle1/goal_distance', Float32, self.goal_distance_callback)
        rospy.Subscriber('turtle1/goal_angle', Float32, self.goal_angle_callback)
        
        self.current_cmd = Twist()

    def goal_distance_callback(self, data):
        self.current_cmd.linear.x = data.data
        self.current_cmd.angular.z = 0.0
        self.publish_cmd_vel()

    def goal_angle_callback(self, data):
        self.current_cmd.angular.z = data.data
        self.current_cmd.linear.x = 0.0
        self.publish_cmd_vel()

    def publish_cmd_vel(self):
        self.velocity_publisher.publish(self.current_cmd)

if __name__ == '__main__':
    try:
        controller = TurtleController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
