#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import BoolStamped
import time

class StopLineSubscriber:
    def __init__(self):
        rospy.init_node('stop_line_subscriber')
        rospy.Subscriber('/duckiebot/stop_line/at_stop_line', BoolStamped, self.callback)

        self.true_count = 0
        self.start_time = None
        self.duration = 1  # Duration to check for continuous 'True' in seconds

    def callback(self, msg):
        if msg.data.data:  # Check if the data field is True
            if self.start_time is None:
                self.start_time = time.time()  # Record the start time

            self.true_count += 1
        else:
            self.true_count = 0  # Reset the count if 'False' is received
            self.start_time = None

        if self.true_count >= int(rospy.get_param("~rate", default=30) * self.duration):
            rospy.loginfo("Continuous 'True' for 1 second detected!")
            rospy.signal_shutdown("Continuous 'True' detected")  # Shutdown the node

if __name__ == '__main__':
    try:
        subscriber = StopLineSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
