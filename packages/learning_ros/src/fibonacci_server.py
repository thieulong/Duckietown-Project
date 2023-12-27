#! /usr/bin/env python3

import rospy
import actionlib
import example_action_server.msg
from example_service.srv import Fibonacci, FibonacciRequest
import sys

def fibonacci_client_service(length):
    rospy.loginfo("Starting service client")
    rospy.loginfo(f"Requesting Fibonacci sequence length of {length}")
    rospy.loginfo("Waiting for service")
    rospy.wait_for_service('calc_fibonacci')
    
    try:
        rospy.loginfo("Service established successfully")
        fibonacci_proxy = rospy.ServiceProxy('calc_fibonacci', Fibonacci)
        
        rospy.loginfo("Sending request to service")
        request = FibonacciRequest(order=length)
        
        rospy.loginfo(f"Waiting for result")
        response = fibonacci_proxy(request)
        
        results = response.sequence
        rospy.loginfo(f"Result for Fibonacci sequence: {results}")
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        
        
def fibonacci_client_action(length):
    rospy.loginfo(f"Starting action client")
    rospy.loginfo(f"Requesting Fibonacci sequence length of {length}")
    
    try:
    
    	client = actionlib.SimpleActionClient('fibonacci', example_action_server.msg.FibonacciAction)

    	rospy.loginfo(f"Waiting for action server")
    	client.wait_for_server()
    
    	rospy.loginfo("Action server established successfully")
    	goal = example_action_server.msg.FibonacciGoal(order=length)
    	
    	rospy.loginfo(f"Sending goal to server")
    	client.send_goal(goal)

    	rospy.loginfo(f"Waiting for result")
    	client.wait_for_result()
    	
    	result = client.get_result()
    	results = result.sequence

    	rospy.loginfo(f"Result for Fibonacci sequence: {results}")

    except rospy.ServiceException as e:
        rospy.logerr("Action server call failed: %s" % e)
    
    
if __name__ == '__main__':
    try:
        rospy.init_node('fibonacci_client_server_action')
        
        length = 3
        
        start_time = rospy.Time.now()
        fibonacci_client_service(length=length)
        end_time = rospy.Time.now()
        time_taken = end_time - start_time
        rospy.loginfo(f"Total time taken: {time_taken.to_sec()} seconds")
        
        start_time = rospy.Time.now()
        fibonacci_client_action(length=length)
        end_time = rospy.Time.now()
        time_taken = end_time - start_time
        rospy.loginfo(f"Total time taken: {time_taken.to_sec()} seconds")
        
        length = 15
        
        start_time = rospy.Time.now()
        fibonacci_client_service(length=length)
        end_time = rospy.Time.now()
        time_taken = end_time - start_time
        rospy.loginfo(f"Total time taken: {time_taken.to_sec()} seconds")
        
        start_time = rospy.Time.now()
        fibonacci_client_action(length=length)
        end_time = rospy.Time.now()
        time_taken = end_time - start_time
        rospy.loginfo(f"Total time taken: {time_taken.to_sec()} seconds")
        
    except rospy.ROSInterruptException:
        rospy.logerr("Program interrupted before completion.", file=sys.stderr)
        pass
