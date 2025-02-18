#!/usr/bin/env python3

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
import tf2_ros

class AprilTagTFBroadcaster:
    def __init__(self):
        """
        Initialize the AprilTagTFBroadcaster node, which listens to AprilTag detections
        and broadcasts their positions and orientations as TF frames.
        """
        rospy.init_node("apriltag_tf_broadcaster", anonymous=True)

        # Initialize the TF broadcaster to send transformations to the TF tree
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscribe to the AprilTag detections topic
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.apriltag_callback)
        rospy.loginfo("Subscribed to /tag_detections topic.")

    def apriltag_callback(self, data):
        """
        Callback to process detections from the AprilTag detection topic.
        For each detected tag, a transformation is broadcasted to the TF tree.

        Args:
        data (AprilTagDetectionArray): A message containing the detected AprilTags
        and their poses in the camera frame.
        """
        # Check if there are any detections
        if not data.detections:
            rospy.loginfo("No AprilTags detected.")
            return

        # Loop through each detected tag
        for detection in data.detections:
            try:
                # Extract the tag ID (assumes a single ID per detection)
                tag_id = detection.id[0]

                # Extract the position and orientation from the detection
                position = detection.pose.pose.pose.position
                orientation = detection.pose.pose.pose.orientation

                # Create a TransformStamped message to define the transform
                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()  # Use the current time
                transform.header.frame_id = "base_link"  # Parent frame (e.g., "map" or "camera_link")
                transform.child_frame_id = f"tag_{tag_id}"  # Child frame named after the tag ID

                # Set the translation (position of the tag relative to the parent frame)
                transform.transform.translation.x = position.x
                transform.transform.translation.y = position.y
                transform.transform.translation.z = position.z

                # Set the rotation (orientation of the tag in quaternion format)
                transform.transform.rotation.x = orientation.x
                transform.transform.rotation.y = orientation.y
                transform.transform.rotation.z = orientation.z
                transform.transform.rotation.w = orientation.w

                # Broadcast the transformation to the TF tree
                self.tf_broadcaster.sendTransform(transform)
                rospy.loginfo(f"Broadcasted transform for tag {tag_id}")

            except AttributeError as e:
                # Handle cases where the detection message might not have the expected structure
                rospy.logwarn(f"Failed to process detection: {e}")

if __name__ == "__main__":
    try:
        # Instantiate the AprilTagTFBroadcaster class and spin to keep the node active
        broadcaster = AprilTagTFBroadcaster()
        rospy.spin()  # Keep the node running to process callbacks
    except rospy.ROSInterruptException:
        # Handle the case where the node is interrupted (e.g., Ctrl+C)
        rospy.loginfo("Node terminated.")

