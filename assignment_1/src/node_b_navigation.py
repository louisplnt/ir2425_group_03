#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
import actionlib

class GlobalNavigation:
    def __init__(self):
        """
        Initialize the global navigation node and its components:
        - ROS action client for `move_base` to send navigation goals.
        - ROS subscriber to listen to `/robot_status` messages.
        - A predefined list of waypoints to navigate through.
        """
        rospy.init_node('global_navigation_node', anonymous=True)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()  # Wait until the move_base action server is available
        rospy.loginfo("Connected to move_base server!")
        self.waypoints = self.define_waypoints()  # Initialize waypoints
        self.stop_navigation = False  # Flag to indicate if navigation should stop

        # Subscribe to the `/robot_status` topic to listen for status updates
        rospy.Subscriber('/robot_status', String, self.status_callback)

    def define_waypoints(self):
        """
        Define a list of waypoints for navigation. Each waypoint is represented
        as a dictionary with:
        - x: x-coordinate
        - y: y-coordinate
        - yaw: orientation in radians (yaw angle)
        """
        return [
            {'x': 0, 'y': -0.55, 'yaw': 0.0},
            {'x': 0.39, 'y': 0, 'yaw': 0.0},
            {'x': 6.3, 'y': 0.17, 'yaw': 0.0},
            {'x': 6.8, 'y': -0.42, 'yaw': 0.0},
            {'x': 9.0, 'y': 0.142, 'yaw': 0.0},
            {'x': 9.0, 'y': 0.8, 'yaw': 0.0},
            {'x': 11.4, 'y': 0.8, 'yaw': 0.0},
            {'x': 12, 'y': 1.5, 'yaw': 0.0},
            {'x': 12.3, 'y': -0.77, 'yaw': 0.0},
            {'x': 11.7, 'y': -0.8, 'yaw': 0.0},
            {'x': 9.0, 'y': -4, 'yaw': 0.0},
            {'x': 7.4, 'y': -3.8, 'yaw': 0.0},
            {'x': 9.0, 'y': -3.8, 'yaw': 0.0},
       
        ]

    def send_goal(self, waypoint):
        """
        Send a navigation goal to the `move_base` action server.
        - The goal specifies the position (x, y) and orientation (yaw) in the `map` frame.
        - Convert the yaw angle to quaternion format for the goal message.

        Args:
        waypoint (dict): A dictionary with x, y, and yaw for the navigation goal.
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # Use the map frame for navigation
        goal.target_pose.header.stamp = rospy.Time.now()  # Current time

        # Set the position for the goal
        goal.target_pose.pose.position.x = waypoint['x']
        goal.target_pose.pose.position.y = waypoint['y']
        goal.target_pose.pose.position.z = 0.0

        # Convert yaw angle to quaternion for orientation
        from tf.transformations import quaternion_from_euler
        quaternion = quaternion_from_euler(0.0, 0.0, waypoint['yaw'])
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        # Send the goal to the move_base server
        rospy.loginfo(f"Sending goal: x={waypoint['x']}, y={waypoint['y']}, yaw={waypoint['yaw']}")
        self.client.send_goal(goal)

        # Wait for the result of the goal
        self.client.wait_for_result()

        # Check if the goal was reached successfully
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached the waypoint!")
        else:
            rospy.logwarn("Failed to reach the waypoint!")

    def status_callback(self, msg):
        """
        Callback to process messages from the `/robot_status` topic.
        - If a specific status message ("All positions updated!") is received,
          stop navigation by setting the `stop_navigation` flag to True.

        Args:
        msg (String): The message received from the `/robot_status` topic.
        """
        if "All positions updated!" in msg.data:
            rospy.loginfo("The research is completed. Stopping navigation.")
            self.stop_navigation = True

    def navigate(self):
        """
        Main navigation logic:
        - Iterate through the list of waypoints and send navigation goals.
        - Check if navigation should stop based on the `stop_navigation` flag.
        """
        rospy.loginfo("Starting global navigation...")
        for waypoint in self.waypoints:
            # Check if navigation should stop
            if self.stop_navigation:
                rospy.loginfo("Navigation stopped due to special status.")
                break

            # Send the navigation goal to the current waypoint
            self.send_goal(waypoint)

        rospy.loginfo("Finished visiting all waypoints.")

if __name__ == '__main__':
    try:
        # Initialize the GlobalNavigation class and start navigating
        navigator = GlobalNavigation()
        navigator.navigate()
    except rospy.ROSInterruptException:
        # Handle ROS interruption (e.g., Ctrl+C)
        rospy.loginfo("Navigation interrupted.")

