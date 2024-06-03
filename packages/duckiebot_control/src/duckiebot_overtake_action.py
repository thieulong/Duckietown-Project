#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState
from std_msgs.msg import Bool

class OvertakeActionNode:
    def __init__(self):
        rospy.init_node('overtake_action_node')

        rospy.Subscriber('/duckiebot/overtaking/activated', Bool, self.overtake_callback)

        self.velocity_pub = rospy.Publisher('/duckiebot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.mode_pub = rospy.Publisher('/duckiebot/fsm_node/mode', FSMState, queue_size=1)
        self.overtake_done_pub = rospy.Publisher('/duckiebot/overtaking/activated', Bool, queue_size=1)

        self.is_overtaking = False

    def enable_lane_controller(self):
        mode_msg = FSMState()
        mode_msg.state = "LANE_FOLLOWING"
        self.mode_pub.publish(mode_msg)
        rospy.loginfo("Duckiebot switched to autonomous")

    def stop_duckiebot(self, duration=0):
        rospy.loginfo("Duckiebot stopped")
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.velocity_pub.publish(cmd_msg)
        rospy.sleep(duration)

    def overtake_action_chain(self):
        rospy.loginfo("Duckiebot overtaking")

        rate = rospy.Rate(10) 

        # Rotate away from current lane
        end_time = rospy.Time.now() + rospy.Duration(0.8)
        while rospy.Time.now() < end_time:
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.0
            cmd_msg.omega = 2.0
            self.velocity_pub.publish(cmd_msg)
            rate.sleep()
        
        # Move forward out of current lane
        end_time = rospy.Time.now() + rospy.Duration(1.2)
        while rospy.Time.now() < end_time:
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.2
            cmd_msg.omega = 0.0
            self.velocity_pub.publish(cmd_msg)
            rate.sleep()

        # Rotate back to face the lane
        end_time = rospy.Time.now() + rospy.Duration(0.8)
        while rospy.Time.now() < end_time:
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.0
            cmd_msg.omega = -2.0
            self.velocity_pub.publish(cmd_msg)
            rate.sleep()

        # Move forward to overtake the obstacle
        end_time = rospy.Time.now() + rospy.Duration(1.5)
        while rospy.Time.now() < end_time:
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.2
            cmd_msg.omega = 0.0
            self.velocity_pub.publish(cmd_msg)
            rate.sleep()

        self.stop_duckiebot()

        self.enable_lane_controller()

        self.overtake_done_pub.publish(Bool(data=False))
        self.is_overtaking = False

    def overtake_callback(self, msg):
        if msg.data and not self.is_overtaking:
            self.is_overtaking = True
            self.overtake_action_chain()

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        overtake_action_node = OvertakeActionNode()
        overtake_action_node.main()
    except rospy.ROSInterruptException:
        pass
