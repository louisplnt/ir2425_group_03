#!/usr/bin/env python3
import rospy
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped

# Global dictionary to store the positions of IDs
ids_positions = {}
last_published_positions = {}  # Last published positions
ids_received = False  # Indicates if the IDs have been received

def generate_default_position():
    """
    Generates a default position for an ID (before receiving transformations).
    Default values are set to zero for x and y coordinates.
    """
    return {"x": 0.0, "y": 0.0}

def ids_callback(msg):
    """
    Callback to receive and store the IDs published by Node A.
    Initializes the `ids_positions` dictionary with default positions for each ID.

    Args:
    msg (String): A message containing comma-separated IDs from Node A.
    """
    global ids_positions, ids_received
    if not ids_received:  # Only initialize once
        rospy.loginfo(f"Received IDs: {msg.data}")
        ids = msg.data.split(',')  # Convert the received string into a list of IDs
        for id_ in ids:
            ids_positions[id_] = generate_default_position()  # Initialize each ID with default positions
        rospy.loginfo(f"Initialized positions: {ids_positions}")
        ids_received = True  # Mark IDs as received to prevent reinitialization

class TFTransformer:
    def __init__(self):
        """
        Initializes the TFTransformer class to manage transformations:
        - Creates a TF buffer and listener to look up transforms.
        - Sets up a periodic timer to check for updates.
        """
        # TF Buffer and Listener for transform lookups
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Run the transformation lookup periodically (every 1 second)
        rospy.Timer(rospy.Duration(1.0), self.lookup_tag_transforms)

    def lookup_tag_transforms(self, event):
        """
        Checks the TF transformations for each known ID and updates their positions
        in the `ids_positions` dictionary.

        Args:
        event: A timer event triggering this function periodically.
        """
        global ids_positions
        for tag_id in ids_positions.keys():
            tf_tag_id = f"tag_{tag_id}"  # Example: ID "1" becomes "tag_1"
            try:
                # Get the transformation from "map" to the tag
                transform = self.tf_buffer.lookup_transform("map", tf_tag_id, rospy.Time(0), rospy.Duration(1.0))
                position = transform.transform.translation

                # Update the position of the corresponding tag
                ids_positions[tag_id] = {
                    "x": position.x,
                    "y": position.y
                }
                rospy.loginfo(f"Updated position for {tag_id}: {ids_positions[tag_id]}")

            except tf2_ros.LookupException as e:
                rospy.logwarn(f"LookupException for {tag_id}: {e}")
            except tf2_ros.ConnectivityException as e:
                rospy.logwarn(f"ConnectivityException for {tag_id}: {e}")
            except tf2_ros.ExtrapolationException as e:
                rospy.logwarn(f"ExtrapolationException for {tag_id}: {e}")

def all_positions_updated():
    """
    Checks if all positions in `ids_positions` have been updated
    (i.e., are different from the default position).

    Returns:
    bool: True if all positions are updated, False otherwise.
    """
    for position in ids_positions.values():
        if position == generate_default_position():  # Compare with default position
            return False
    return True

def main():
    """
    Main function to initialize the node, manage IDs and transformations,
    and publish updated positions to the `/robot_status` topic.
    """
    global ids_positions, ids_received, last_published_positions

    # Initialize the ROS node
    rospy.init_node("node_b_tf_fusion", anonymous=True)
    rospy.loginfo("Node B with TF Fusion started!")

    # Subscribe to the `/apriltags_ids` topic to receive IDs from Node A
    rospy.Subscriber('/apriltags_ids', String, ids_callback)

    # Publisher for broadcasting position updates
    status_publisher = rospy.Publisher('/robot_status', String, queue_size=10)

    # Instantiate the TFTransformer to handle transformations
    transformer = TFTransformer()

    # Main loop to publish updated positions
    rate = rospy.Rate(1)  # Loop frequency: 1 Hz
    while not rospy.is_shutdown():
        if ids_received:  # Only proceed if IDs have been received
            # Check if any positions have changed since the last published state
            if ids_positions != last_published_positions:
                status_message = f"Tracking AprilTags: {ids_positions}"
                status_publisher.publish(status_message)  # Publish the updated positions
                rospy.loginfo(f"Status published: {status_message}")
                last_published_positions = ids_positions.copy()  # Update the last published state

            # Check if all positions have been updated
            if all_positions_updated():
                special_message = "All positions updated!"
                status_publisher.publish(special_message)  # Publish a special status message
                rospy.loginfo(f"Special Status published: {special_message}")

        rate.sleep()  # Wait for the next iteration

if __name__ == "__main__":
    try:
        # Run the main function
        main()
    except rospy.ROSInterruptException:
        # Handle shutdown gracefully
        rospy.loginfo("Node terminated.")

