#!/usr/bin/env python3

import rospy
import math
from turtlesim.msg import Pose
from turtlesim_helper.msg import UnitsLabelled

class DistanceReader:
    def __init__(self):
        self.distance = 0
        self.prev_pose = None
        self.pub_distance = rospy.Publisher("/turtle1/distance", UnitsLabelled, queue_size=10)
        rospy.Subscriber("/turtle1/pose", Pose, self.subscribe_function)
    
    def subscribe_function(self, data):
        if self.prev_pose is not None:
            # Calculate the Euclidean distance between the current and previous pose
            dx = data.x - self.prev_pose.x
            dy = data.y - self.prev_pose.y
            distance = math.sqrt(dx**2 + dy**2)

            # Update the total distance
            self.distance += distance

            # Create a UnitsLabelled message with the total distance in meters
            msg = UnitsLabelled()
            msg.value = self.distance
            msg.units = "meters"

            # Publish the message
            self.pub_distance.publish(msg)

        self.prev_pose = data

if __name__ == '__main__':
    rospy.init_node('distance_travelled_node', anonymous=False)
    distance_reader = DistanceReader()
    rospy.spin()
