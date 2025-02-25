#!/usr/bin/env python3

import rospy
import tf2_ros
from std_msgs.msg import String
import tf2_geometry_msgs

"""

   Node_b
   
   Use node_b_camera and node_b_nodesDetection to locate the apritags. 
   Recieve the ids and positions, and transfom it in to coordonates


"""

# Global dictionary to store the positions of IDs
ids_positions = {}
last_published_positions = {}  # Last published positions

def ids_callback(msg):
    """
    Callback to receive and store the IDs and positions published by node_b_nodesDetection.
    """
    global ids_positions
    rospy.loginfo(f"Received IDs and positions: {msg.data}")
    tags = msg.data.split(';')  # Split the message into individual tag entries

    for tag in tags:
        try:
            tag_id, x, y, z = tag.split(',')  # Extract ID and positions
            tag_id = tag_id.strip()
            position = {"x": float(x), "y": float(y), "z": float(z)}
            ids_positions[tag_id] = position
            rospy.loginfo(f"Updated tag: ID={tag_id}, Position={position}")
        except ValueError:
            rospy.logwarn(f"Invalid tag format: {tag}")

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
        tag_id (str): The ID of the tag to transform.

        Returns:
        dict: The updated position if the transformation succeeds, else None.
        """
        global ids_positions
        tf_tag_id = f"tag_{tag_id}"  # Example: ID "1" becomes "tag_1"
        try:
            # Get the transformation from "map" to the tag
            transform = self.tf_buffer.lookup_transform("map", tf_tag_id, rospy.Time(0), rospy.Duration(1.0))
            position = transform.transform.translation

            # Update the position of the corresponding tag
            transformed_position = {"x": position.x, "y": position.y}
            ids_positions[tag_id] = transformed_position
            return transformed_position

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to transform tag {tag_id}: {e}")
            return None

def main():
    """
    Main function to initialize the node, manage IDs and transformations,
    and publish updated positions to the `/robot_status` topic.
    """
    global ids_positions, last_published_positions

    # Initialize the ROS node
    rospy.init_node("node_b_tf_fusion", anonymous=True)
    rospy.loginfo("Node B with TF Fusion started!")

    # Subscribe to the `/apriltags_ids_positions` topic to receive IDs and positions from node_b_nodesDetection
    rospy.Subscriber('/apriltags_ids_positions', String, ids_callback)

    # Publisher for broadcasting position updates (if needed)
    status_publisher = rospy.Publisher('/robot_status', String, queue_size=10)

    # Instantiate the TFTransformer to handle transformations
    transformer = TFTransformer()

    # Main loop to process the IDs and transform positions
    rate = rospy.Rate(1)  # Loop frequency: 1 Hz
    while not rospy.is_shutdown():
        # Only proceed if IDs have been received
        if ids_positions:
            # Create a copy of the dictionary to avoid issues with size changes
            current_positions = dict(ids_positions)

            for tag_id, position in current_positions.items():
                # Transform the position (if needed)
                transformed_position = transformer.lookup_tag_transform(tag_id)
                if transformed_position:
                    # Display the transformed position in the terminal
                    rospy.loginfo(f"Transformed position for tag {tag_id}: {transformed_position}")
                    # Optionally, publish the status
                    status_message = f"Tag {tag_id}: {transformed_position}"
                    status_publisher.publish(status_message)
                else:
                    rospy.loginfo(f"Tag {tag_id} transformation failed.")

        rate.sleep()  # Wait for the next iteration

if __name__ == "__main__":
    try:
        # Run the main function
        main()
    except rospy.ROSInterruptException:
        # Handle shutdown gracefully
        rospy.loginfo("Node terminated.")

