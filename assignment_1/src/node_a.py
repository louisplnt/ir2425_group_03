#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from tiago_iaslab_simulation.srv import Objs

# Global variable to indicate whether to stop publishing
stop_publishing = False

def status_callback(msg):
    """
    Callback to receive updates from Node B. This function listens to messages
    on the `/robot_status` topic to determine if Node B has received and processed
    the AprilTag IDs.

    Args:
    msg (String): Message received from Node B via the `/robot_status` topic.
    """
    global stop_publishing
    rospy.loginfo(f"Received status update from Node B: {msg.data}")
    
    # Check if the message indicates that Node B has successfully processed the IDs
    if "Tracking AprilTag" in msg.data:
        # Stop publishing IDs to prevent unnecessary network traffic
        stop_publishing = True

def request_apriltag_ids():
    """
    Makes a service call to retrieve the AprilTag IDs from a ROS service.

    Returns:
    list: A list of AprilTag IDs retrieved from the service, or an empty list
    if the service call fails.
    """
    rospy.wait_for_service('/apriltag_ids_srv')  # Wait until the service is available
    try:
        # Create a service proxy for the `/apriltag_ids_srv` service
        get_ids = rospy.ServiceProxy('/apriltag_ids_srv', Objs)
        # Call the service with `ready=True` to request the IDs
        response = get_ids(ready=True)
        rospy.loginfo(f"AprilTag IDs retrieved: {response.ids}")
        return response.ids
    except rospy.ServiceException as e:
        # Log an error message if the service call fails
        rospy.logerr(f"Service call failed: {e}")
        return []

def main():
    """
    Main function for Node A. This node is responsible for:
    - Retrieving AprilTag IDs from a service.
    - Publishing these IDs to Node B via the `/apriltags_ids` topic.
    - Stopping publication once Node B confirms receipt of the IDs.
    """
    global stop_publishing

    # Initialize the ROS node
    rospy.init_node('node_a', anonymous=True)
    rospy.loginfo("Node A started!")

    # Subscribe to the `/robot_status` topic to receive updates from Node B
    rospy.Subscriber('/robot_status', String, status_callback)

    # Create a publisher to send the AprilTag IDs to Node B
    tag_ids_publisher = rospy.Publisher('/apriltags_ids', String, queue_size=10)

    # Retrieve the AprilTag IDs via a service call
    ids = request_apriltag_ids()

    # Publish the IDs periodically until Node B confirms their receipt
    rate = rospy.Rate(1)  # Set the publication rate to 1 Hz
    
    while not rospy.is_shutdown():
        if stop_publishing:  # Stop publishing if instructed by Node B
            rospy.loginfo("Stopping publication as instructed by Node B.")
            while not rospy.is_shutdown():
                rospy.spin()  # Keep the node alive to handle callbacks
        else:
            if ids:  # Only publish if IDs were successfully retrieved
                # Publish the IDs as a comma-separated string
                tag_ids_publisher.publish(','.join(map(str, ids)))
                rospy.loginfo(f"Published AprilTag IDs: {','.join(map(str, ids))}")
        rate.sleep()  # Wait until the next loop iteration

if __name__ == '__main__':
    """
    Entry point for the script. Executes the main function.
    """
    main()

