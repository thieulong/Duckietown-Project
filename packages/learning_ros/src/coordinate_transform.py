#!/usr/bin/env python3

from math import radians, sin, cos
import numpy as np
import rospy
from duckietown_msgs.msg import Vector2D

class TransformCoordinate:
    def __init__(self):
        rospy.Subscriber("/sensor_coord", Vector2D, self.transform_coordinates)
        self.robot_coord_pub = rospy.Publisher("/robot_coord", Vector2D, queue_size=10)
        self.world_coord_pub = rospy.Publisher("/world_coord", Vector2D, queue_size=10)
        
        self.robot_pos = np.array([5, 3])
        self.robot_deg = 135
        self.robot_rad = np.deg2rad(self.robot_deg)

        self.sensor_pos = np.array([-1, 0])
        self.sensor_deg = -180
        self.sensor_rad = np.deg2rad(self.sensor_deg)

    def transform_coordinates(self, coord_msg):
        robot_coordinate = self.robot_coord_transform(coord_msg)
        if robot_coordinate: self.robot_coord_pub.publish(robot_coordinate)
        world_coordinate = self.world_coord_transform(robot_coordinate, coord_msg)
        if world_coordinate: self.world_coord_pub.publish(world_coordinate)

    def robot_coord_transform(self, coord_msg):
        RpS = self.sensor_pos.reshape(-1,1)

        RrS = np.array([[cos(self.sensor_rad),-sin(self.sensor_rad)],
                        [sin(self.sensor_rad), cos(self.sensor_rad)]])
        
        RtS = np.zeros([3,3])

        RtS[0][0] = RrS[0][0]
        RtS[0][1] = RrS[0][1]
        RtS[0][2] = RpS[0][0]

        RtS[1][0] = RrS[1][0]
        RtS[1][1] = RrS[1][1]
        RtS[1][2] = RpS[1][0]

        RtS[2][0] = 0
        RtS[2][1] = 0
        RtS[2][2] = 1

        SpP = np.array([coord_msg.x, coord_msg.y, 1]).reshape(-1,1)
        
        RpP = np.dot(RtS, SpP)
        
        robot_coord = Vector2D(x=RpP[0][0], y=RpP[1][0])

        return robot_coord

    def world_coord_transform(self, robot_coord, coord_msg):
        WpR = self.robot_pos.reshape(-1,1)

        WrR = np.array([[cos(self.robot_rad),-sin(self.robot_rad)],
                        [sin(self.robot_rad), cos(self.robot_rad)]])
        
        WtR = np.zeros([3,3])

        WtR[0][0] = WrR[0][0]
        WtR[0][1] = WrR[0][1]
        WtR[0][2] = WpR[0][0]

        WtR[1][0] = WrR[1][0]
        WtR[1][1] = WrR[1][1]
        WtR[1][2] = WpR[1][0]

        WtR[2][0] = 0
        WtR[2][1] = 0
        WtR[2][2] = 1

        RpP = np.array([robot_coord.x, robot_coord.y, 1]).reshape(-1,1)
        
        WpP = np.dot(WtR, RpP)
        
        world_coord = Vector2D(x=WpP[0][0], y=WpP[1][0])

        return world_coord
    
if __name__ == '__main__':
    rospy.init_node('transform_coordinate', anonymous=True)
    transform_coordinate = TransformCoordinate()
    rospy.spin()