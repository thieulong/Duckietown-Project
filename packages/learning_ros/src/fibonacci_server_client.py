#!/usr/bin/env python3

import rospy
from example_service.srv import Fibonacci, FibonacciRequest

def fibonacci_client(order):
    start_time = rospy.Time.now()
    rospy.loginfo("Waiting for service")
    rospy.wait_for_service('calc_fibonacci')
    try:
        rospy.loginfo("Service established successfully")
        fibonacci_proxy = rospy.ServiceProxy('calc_fibonacci', Fibonacci)
        request = FibonacciRequest(order=order)
        response = fibonacci_proxy(request)
        end_time = rospy.Time.now()
        time_taken = end_time - start_time
        rospy.loginfo(f"Total time taken: {time_taken.to_sec()} seconds")
        return response.sequence
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    try:
        rospy.init_node('fibonacci_client_server')

        length = 3
        fibonacci_result = fibonacci_client(order=length)

        if fibonacci_result is not None:
            rospy.loginfo(f"Fibonacci sequence length of {length}: {fibonacci_result}")
        
        length = 15
        fibonacci_result = fibonacci_client(order=length)

        if fibonacci_result is not None:
            rospy.loginfo(f"Fibonacci sequence length of {length}: {fibonacci_result}")

    except rospy.ROSInterruptException:
    	rospy.logerr("[ERROR] Program interrupted before completion.", file=sys.stderr)
        pass

