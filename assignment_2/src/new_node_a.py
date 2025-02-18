#!/usr/bin/env python3

import rospy
from tiago_iaslab_simulation.srv import Coeffs

def request_coeffs():
    rospy.init_node("node_a")
    rospy.wait_for_service("/straight_line_srv")

    try:
        service_proxy = rospy.ServiceProxy("/straight_line_srv", Coeffs)
        response = service_proxy(True)
        m, q = response.coeffs
        rospy.loginfo(f"Received coefficients: m = {m}, q = {q}")

        placing_routine(m, q)

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed {e}")

def placing_routine(m, q):
    rospy.loginfo(f"Execution placing routine with m = {m} and q = {q}")

if __name__ == "__main__":
    request_coeffs()