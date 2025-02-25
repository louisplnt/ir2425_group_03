#!/usr/bin/env python3

import rospy
import tf2_ros
from std_msgs.msg import String
import tf2_geometry_msgs
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PointStamped  # For point transformations
from apriltag_ros.msg import AprilTagDetectionArray

"""
   Node_b
   
   Use node_b_camera to locate the apriltags. 
   Receives the ids and positions, transforms them into coordinates,
   and sends the transformed positions to node_c upon request.
"""

# Global Lists
list_id_detected = []                   # Stores IDs of all detected AprilTags
list_apriltags_raw = [None] * 11         # Stores raw positions of detected AprilTags
list_apriltags_transfom = [None] * 11    # Stores transformed positions to be sent to node_c
transformer = None                      # Declare the transformer globally


def apriltag_callback(data):
    """
    Callback to process detections from the AprilTag detection topic.
    Checks if the ID is new, stores raw position, and transforms it.
    """
    global list_id_detected, list_apriltags_raw, list_apriltags_transfom, transformer
    
    # Check if there are any detections
    if not data.detections:
        return

    # Loop through each detection
    for detection in data.detections:
        try:
            # Extract the tag ID (assumes a single ID per detection)
            tag_id = detection.id[0]

            # If the tag is already detected, skip processing
            if tag_id in list_id_detected:
                continue

            # Extract the raw position from the detection
            position = detection.pose.pose.pose.position

            # Store the raw position in list_apriltags_raw
            list_apriltags_raw[tag_id] = [position.x, position.y, position.z]
            rospy.loginfo(f"Raw Position Stored - ID {tag_id}: {list_apriltags_raw[tag_id]}")

            # Add the ID to the list of detected tags
            list_id_detected.append(tag_id)

            # Transform the position and store it in list_apriltags_transfom
            transformed_position = transformer.lookup_tag_transform(tag_id)
            if transformed_position:
                list_apriltags_transfom[tag_id] = [
                    transformed_position["x"],
                    transformed_position["y"],
                    transformed_position["z"]
                ]
                rospy.loginfo(f"Transformed Position Stored - ID {tag_id}: {list_apriltags_transfom[tag_id]}")
            else:
                rospy.logwarn(f"Failed to transform tag {tag_id}")

        except AttributeError as e:
            rospy.logwarn(f"Failed to process detection: {e}")


class TFTransformer:
    def __init__(self):
        """
        Initializes the TFTransformer class to manage transformations:
        - Creates a TF buffer and listener to look up transforms.
        """
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def lookup_tag_transform(self, tag_id):
        """
        Looks up the transformation for a specific tag and updates its position.

        Args:
        tag_id (int): The ID of the tag to transform.

        Returns:
        dict: The updated position if the transformation succeeds, else None.
        """
        global list_apriltags_raw
        tf_tag_id = f"tag_{tag_id}"  # Example: ID "1" becomes "tag_1"
        try:
            # Get the transformation from "map" to the tag's camera frame
            transform = self.tf_buffer.lookup_transform("map", tf_tag_id, rospy.Time(0), rospy.Duration(1.0))

            # Get the raw position from list_apriltags_raw
            raw_position = list_apriltags_raw[tag_id]
            if raw_position is None:
                rospy.logwarn(f"No raw position found for tag {tag_id}")
                return None

            # Create a PointStamped message for the original position
            point = PointStamped()
            point.header.frame_id = tf_tag_id
            point.point.x = raw_position[0]
            point.point.y = raw_position[1]
            point.point.z = raw_position[2]

            # Transform the point into the "map" frame
            transformed_point = tf2_geometry_msgs.do_transform_point(point, transform)

            # Return the transformed position
            transformed_position = {
                "x": transformed_point.point.x,
                "y": transformed_point.point.y,
                "z": transformed_point.point.z  # Keep the z coordinate
            }
            return transformed_position

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to transform tag {tag_id}: {e}")
            return None


# Handler Function for Trigger Service
def handle_ping_request(req):
    """
    Handles the service request from node_c and responds with the list of transformed AprilTags.
    """
    global list_apriltags_transfom
    rospy.loginfo("AprilTags requested by node_c")
    rospy.loginfo("Providing AprilTags to node_c:")

    # Format "id,x,y,z ; id,x,y,z ; ..."
    string_apriltags = ""
    for i in range(11):
        if list_apriltags_transfom[i] is not None:
            string_apriltags += f"{i},{list_apriltags_transfom[i][0]},{list_apriltags_transfom[i][1]},{list_apriltags_transfom[i][2]};"
            rospy.loginfo(f"- apriltag_{i}:{list_apriltags_transfom[i][0]},{list_apriltags_transfom[i][1]},{list_apriltags_transfom[i][2]}")
    # Take out the last ";"
    if len(string_apriltags) > 0:
        string_apriltags = string_apriltags[:-1]

    return TriggerResponse(success=True, message=string_apriltags)


def main():
    """
    Main function to initialize the node, manage IDs and transformations,
    and respond to node_c's requests.
    """
    global transformer

    # Initialize the ROS node
    rospy.init_node("node_b_main", anonymous=True)
    rospy.loginfo("Node B main started!")

    # Instantiate the TFTransformer to handle transformations
    transformer = TFTransformer()

    # Subscribe to the AprilTag detection topic
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, apriltag_callback)

    # Advertise the ping service
    rospy.Service('/get_apriltags_detected', Trigger, handle_ping_request)
    rospy.loginfo("Service /get_apriltags_detected is ready.")

    rospy.spin()  # Keep the node running


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")

