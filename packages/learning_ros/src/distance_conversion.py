#!/usr/bin/env python3

import rospy
from turtlesim_helper.msg import UnitsLabelled

class ConvertDistance:
    def __init__(self):
        self.distance_meters = 0
        self.distance_feet = 0
        self.distance_smoots = 0
        self.pubdistance_measurements = rospy.Publisher("turtle1/converted_distance", UnitsLabelled, queue_size=10)
        rospy.Subscriber("turtle1/distance", UnitsLabelled, self.convert_distance)

        # Define a default conversion unit parameter and rate
        self.conversion_unit = rospy.get_param("/conversion_unit", "smoots")
        self.rate = rospy.Rate(1)  # Adjust the rate as needed

    def convert_distance(self, data):
        # Read the parameter to determine the desired unit
        desired_unit = rospy.get_param("/conversion_unit", "smoots")

        # Perform conversion based on the desired unit
        if desired_unit == "meters":
            self.distance_meters = data.value * 1

        elif desired_unit == "feet":
            self.distance_feet = data.value * 3.28084

        elif desired_unit == "smoots":
            self.distance_smoots = data.value * 5.5833

        # Publish the converted distance
        converted_distance = UnitsLabelled()
        converted_distance.value = getattr(self, f"distance_{desired_unit}")
        converted_distance.units = desired_unit
        self.pubdistance_measurements.publish(converted_distance)

        # Rate limit the loop
        # self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('convert_distance', anonymous=True)
    convert_distance = ConvertDistance()
    rospy.spin()
