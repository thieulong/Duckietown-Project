#! /usr/bin/env python3

import rospy
import actionlib
import example_action_server.msg
import sys

def fibonacci_client(length):
    start_time = rospy.Time.now()
    rospy.loginfo(f"Starting client ...")
    client = actionlib.SimpleActionClient('fibonacci', example_action_server.msg.FibonacciAction)

    rospy.loginfo(f"Waiting for server ...")
    client.wait_for_server()
    
    goal = example_action_server.msg.FibonacciGoal(order=length)
    send_goal_time = rospy.Time.now()
    rospy.loginfo(f"Sending goal to server ...")
    client.send_goal(goal)

    rospy.loginfo(f"Waiting for result ...")
    client.wait_for_result()

    end_time = rospy.Time.now()
    result = client.get_result()
    print(f"[INFO] [{rospy.get_rostime()}] Result for Fibonacci sequence with length of {length}:", ', '.join([str(n) for n in result.sequence]))

    time_taken = end_time - start_time
    rospy.loginfo(f"Total time taken: {time_taken.to_sec()} seconds")

if __name__ == '__main__':
    try:
        rospy.init_node('fibonacci_client_action')
        
        length = 3
        fibonacci_client(length=length)
        
        length = 15
        fibonacci_client(length=length)
        
    except rospy.ROSInterruptException:
        rospy.logerr("[ERROR] Program interrupted before completion.", file=sys.stderr)
        pass

