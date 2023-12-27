#!/usr/bin/env python3

import rospy
import time

def set_conversion_unit(param_value):
    rospy.set_param("/conversion_unit", param_value)
    rospy.loginfo(f"Set conversion_unit to {param_value}")

def parameter_setting_node():
    rospy.init_node('parameter_setting_node', anonymous=True)

    while not rospy.is_shutdown():
        # Set conversion_unit to "Meters"
        set_conversion_unit("meters")
        time.sleep(5)

        # Set conversion_unit to "Feet"
        set_conversion_unit("feet")
        time.sleep(5)

        # Set conversion_unit to "Smoots"
        set_conversion_unit("smoots")
        time.sleep(5)

if __name__ == '__main__':
    try:
        parameter_setting_node()
    except rospy.ROSInterruptException:
        pass
