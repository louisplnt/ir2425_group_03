#!/usr/bin/env python3

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose
from std_msgs.msg import String

"""
   node_b_nodesDetection


   Locate the apriltags and send the info to node_b_main
   

"""

class AprilTagTFBroadcaster:
    def __init__(self):
        """
        Initialize the AprilTagTFBroadcaster node, which listens to AprilTag detections
        and stores their IDs and positions for regular display.
        """
        rospy.init_node("apriltag_tf_broadcaster", anonymous=True)

        # Initialize storage for detected tags
        self.detected_ids = []  # List to store IDs of detected tags
        self.detected_positions = []  # List to store positions of detected tags

        # Publisher to send IDs and positions to node_b_main
        self.position_publisher = rospy.Publisher("/apriltags_ids_positions", String, queue_size=10)

        # Subscribe to the AprilTag detections topic
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.apriltag_callback)
        rospy.loginfo("Subscribed to /tag_detections topic.")

        # Set up a timer to display and send the IDs and positions periodically
        rospy.Timer(rospy.Duration(1), self.publish_detected_ids)  # Publish every 1 second

    def apriltag_callback(self, data):
        """
        Callback to process detections from the AprilTag detection topic.
        For each detected tag, its ID and position are stored.

        Args:
        data (AprilTagDetectionArray): A message containing the detected AprilTags
        and their poses in the camera frame.
        """
        if not data.detections:
            return

        for detection in data.detections:
            try:
                # Extract the tag ID (assumes a single ID per detection)
                tag_id = detection.id[0]

                # Extract the position from the detection
                position = detection.pose.pose.pose.position

                # Add the ID and position to the respective lists if not already added
                if tag_id not in self.detected_ids:
                    self.detected_ids.append(tag_id)
                    self.detected_positions.append((position.x, position.y, position.z))
                    rospy.loginfo(f"Stored ID {tag_id} with position {position.x}, {position.y}, {position.z}")

            except AttributeError as e:
                rospy.logwarn(f"Failed to process detection: {e}")

    def publish_detected_ids(self, event):
        """
        Periodically send the list of detected IDs and their positions to node_b_main.
        """
        if not self.detected_ids:
            return

        # Prepare the data to be sent as a string in the format: ID,x,y,z;ID2,x2,y2,z2;
        data_to_send = ""
        for tag_id, position in zip(self.detected_ids, self.detected_positions):
            data_to_send += f"{tag_id},{position[0]},{position[1]},{position[2]};"

        # Publish the data as a String message
        self.position_publisher.publish(data_to_send)
        rospy.loginfo(f"Published detected IDs and positions: {data_to_send}")

if __name__ == "__main__":
    try:
        # Instantiate the AprilTagTFBroadcaster class and spin to keep the node active
        broadcaster = AprilTagTFBroadcaster()
        rospy.spin()  # Keep the node running to process callbacks
    except rospy.ROSInterruptException:
        # Handle the case where the node is interrupted (e.g., Ctrl+C)
        rospy.loginfo("Node terminated.")

