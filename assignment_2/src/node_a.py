#!/usr/bin/env python3

import rospy
from tiago_iaslab_simulation.srv import Coeffs, CoeffsRequest
from std_srvs.srv import Trigger, TriggerResponse


"""
   Node_a
   
   Find the m and q coefficients with /straight_line_srv and send it to node_c when he request.




"""

# Global variables to store coefficients
m, q = None, None

def get_line_coefficients():
    global m, q
    rospy.wait_for_service('/straight_line_srv')

    try:
        straight_line_service = rospy.ServiceProxy('/straight_line_srv', Coeffs)
        request = CoeffsRequest(ready=True)
        response = straight_line_service(request)

        if len(response.coeffs) == 2:
            m, q = response.coeffs
            rospy.loginfo(f"Received line coefficients: m = {m}, q = {q}")
        else:
            rospy.logerr(f"Unexpected number of coefficients: {response.coeffs}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def handle_coeff_request(req):
    """Service handler to provide coefficients to node_c."""
    global m, q
    if m is not None and q is not None:
        rospy.loginfo("Providing coefficients to node_c")
        return TriggerResponse(success=True, message=f"{m},{q}")
    else:
        rospy.logwarn("Coefficients not available yet.")
        return TriggerResponse(success=False, message="Coefficients not ready")

def main():
    global m, q
    rospy.init_node('line_handler_node')

    # Service to provide coefficients to node_c
    rospy.Service('/get_coefficients', Trigger, handle_coeff_request)

    # Retrieve coefficients
    get_line_coefficients()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted before completion.")

