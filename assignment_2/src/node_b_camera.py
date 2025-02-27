#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from control_msgs.msg import PointHeadAction, PointHeadGoal
from actionlib import SimpleActionClient

class LookToPoint:
    def __init__(self):
        """
        Initialize the `LookToPoint` node, responsible for controlling the camera view
        and making the robot head point towards specific points.
        """
        rospy.init_node("look_to_point", anonymous=True)

        # Store camera parameters
        self.camera_intrinsics = None  # Camera intrinsic parameters matrix
        self.latest_image_stamp = None  # Timestamp of the latest received image

        # Initialize an action client to control the robot head
        self.point_head_client = SimpleActionClient("/head_controller/point_head_action", PointHeadAction)
        rospy.loginfo("Waiting for PointHeadAction server...")
        self.point_head_client.wait_for_server()  # Wait until the action server is ready
        rospy.loginfo("Connected to PointHeadAction server.")

        # Initialize an OpenCV bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()

        # Subscribe to the camera image and camera info topics
        rospy.Subscriber("/xtion/rgb/image_raw", Image, self.image_callback)
        rospy.Subscriber("/xtion/rgb/camera_info", CameraInfo, self.camera_info_callback)

        # OpenCV window for displaying the camera feed
        self.window_name = "TIAGo Camera View"
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)

    def camera_info_callback(self, msg):
        """
        Callback function to process the camera info messages. Extracts and stores the camera's intrinsic parameters.

        Args:
        msg (CameraInfo): A ROS CameraInfo message containing the camera parameters.
        """
        self.camera_intrinsics = np.zeros((3, 3))  # Initialize a 3x3 matrix for intrinsics
        self.camera_intrinsics[0, 0] = msg.K[0]  # Focal length in x (fx)
        self.camera_intrinsics[1, 1] = msg.K[4]  # Focal length in y (fy)
        self.camera_intrinsics[0, 2] = msg.K[2]  # Principal point x-coordinate (cx)
        self.camera_intrinsics[1, 2] = msg.K[5]  # Principal point y-coordinate (cy)
        self.camera_intrinsics[2, 2] = 1.0  # Homogeneous coordinate
        rospy.loginfo("Camera intrinsics received and stored.")

    def image_callback(self, msg):
        """
        Callback function to process image messages from the camera. Converts the ROS image to OpenCV format
        and displays it in a window.

        Args:
        msg (Image): A ROS Image message containing the camera image data.
        """
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image_stamp = msg.header.stamp  # Save the timestamp of the image
            cv2.imshow(self.window_name, cv_image)  # Display the image in an OpenCV window
            cv2.waitKey(15)  # Refresh the window
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")  # Log any conversion errors

    def look_downward(self):
        """
        Send a command to make the robot head point downward, simulating looking straight down at the ground.
        """
        rospy.loginfo("Moving the camera to look downward.")

        # Define a target point slightly forward and downward
        point = PointStamped()
        point.header.frame_id = "/xtion_rgb_optical_frame"  # Define the reference frame
        point.header.stamp = rospy.Time.now()  # Set the timestamp
        point.point.x = 0.0  # Target directly below the robot
        point.point.y = 1.0  # Slightly forward for better view
        point.point.z = 0.0  # At camera height

        # Create a PointHeadGoal to define the head movement
        goal = PointHeadGoal()
        goal.pointing_frame = "/xtion_rgb_optical_frame"  # Frame to control head pointing
        goal.pointing_axis.x = 0.0  # Define the pointing direction (z-axis of the frame)
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 1.0
        goal.min_duration = rospy.Duration(1.0)  # Minimum duration for the movement
        goal.max_velocity = 0.25  # Limit the head movement speed
        goal.target = point  # Target point to look at

        # Send the goal to the action server
        self.point_head_client.send_goal(goal)
        rospy.loginfo("Sent downward pointing goal.")
        self.point_head_client.wait_for_result()  # Wait for the action to complete

    def run(self):
        """
        Keep the node running and process callbacks (e.g., image display and camera info).
        """
        rospy.spin()  # Wait for callbacks to be called
        cv2.destroyAllWindows()  # Close any OpenCV windows when the node shuts down

if __name__ == "__main__":
    try:
        # Instantiate the LookToPoint class and run the node
        look_to_point = LookToPoint()
        look_to_point.look_downward()  # Command the robot to look downward initially
        look_to_point.run()  # Start processing callbacks
    except rospy.ROSInterruptException:
        rospy.loginfo("Look to Point node terminated.")

