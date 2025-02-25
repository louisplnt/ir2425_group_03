#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
import actionlib
from tf.transformations import quaternion_from_euler
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
import sys
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_commander import MoveGroupCommander
from moveit_commander import PlanningSceneInterface
from std_srvs.srv import Trigger



class Navigator:
    def __init__(self):
        rospy.init_node('global_navigation_node', anonymous=True)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.stop_navigation = False  # Flag to indicate if navigation should stop

        # Subscribe to the `/robot_status` topic to listen for status updates
        rospy.Subscriber('/robot_status', String, self.status_callback)

        # Store AprilTag data
        self.april_tags_data = []  # List to store the AprilTag IDs and their coordinates

    
    def send_goal(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the position for the goal
        goal.target_pose.pose.position.x = waypoint['x']
        goal.target_pose.pose.position.y = waypoint['y']
        goal.target_pose.pose.position.z = 0.0

        # Convert yaw angle to quaternion for orientation
        quaternion = quaternion_from_euler(0.0, 0.0, waypoint['yaw'])
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        # Send the goal to the move_base server
        rospy.loginfo(f"Navigator: go to x={waypoint['x']}, y={waypoint['y']}, yaw={waypoint['yaw']}")
        self.client.send_goal(goal)

        # Wait for the result of the goal
        self.client.wait_for_result()

        # Check if the goal was reached successfully
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigator: Reached the waypoint!")
        else:
            rospy.logwarn("Failed to reach the waypoint!")

    def status_callback(self, msg):
        if "All positions updated!" in msg.data:
            rospy.loginfo("The research is completed. Stopping navigation.")
            self.stop_navigation = True
            
    def select_and_generate_waypoints(self, tags):
        # Sort tags by X coordinate in descending order
        sorted_tags = sorted(tags, key=lambda x: x['x'], reverse=True)
        
        # Select the top 3 tags with the largest X values
        top_tags = sorted_tags[:3]
        
        # Create waypoints with a fixed yaw value
        waypoints = []
        arm_len = 0.8
        for tag in top_tags:
            waypoint = {
                'x': tag['x'],
                'y': tag['y'] + arm_len,
                'yaw': 3.14  # Fixed yaw value
            }
            waypoints.append(waypoint)
        
        return waypoints



class AprilTagsRequester:
    def __init__(self):
        pass

    def request_april_tags(self):
        rospy.wait_for_service('/get_april_tags', timeout=5.0)
        try:
            get_april_tags = rospy.ServiceProxy('/get_april_tags', Trigger)
            response = get_april_tags()  

            if response.success:
                tags = []
                tag_data = response.message.strip(';').split(';')
                for tag in tag_data:
                    tag_id, x, y, z = tag.split(',')
                    tags.append({'id': tag_id, 'x': float(x), 'y': float(y), 'z': float(z)})
                return tags
            else:
                rospy.logwarn("Fail to get data from AprilTag.")
                return []
        except rospy.ServiceException as e:
            rospy.logerr(f"Service fail : {e}")
            return []







class ArmManipulator:
    def __init__(self):
        # Initialisation of MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        # Commande group of the arm
        self.group_arm = moveit_commander.MoveGroupCommander("arm")
        rospy.loginfo("Arm: init finish")
        
    
    def gripper_open(self):
        # Gripper Init
        # Connexion with the action server of the gripper
        grip_client = SimpleActionClient('/parallel_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        grip_client.wait_for_server()
        
        gripper_goal = FollowJointTrajectoryGoal()
        gripper_goal.trajectory.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        
        # Set the positions and trajectory
        point = JointTrajectoryPoint()
        point.positions = [0.8, 0.8]                  # Open
        point.time_from_start = rospy.Duration(1.0)   # 1sec to open
        
        gripper_goal.trajectory.points = [point]
        gripper_goal.trajectory.header.stamp = rospy.Time.now()  # Start
        
        grip_client.send_goal(gripper_goal)
        rospy.sleep(1)                     #Gripper wait for the gripper action to be finish
        rospy.loginfo("Gripper: open")

        
            
    def gripper_close(self):
        # Gripper Init
        # Connexion with the action server of the gripper
        grip_client = SimpleActionClient('/parallel_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        grip_client.wait_for_server()
        
        gripper_goal = FollowJointTrajectoryGoal()
        gripper_goal.trajectory.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        
        # Set the positions and trajectory
        point = JointTrajectoryPoint()
        point.positions = [0.01, 0.01]                # Close
        point.time_from_start = rospy.Duration(1.0)   # 1sec to close
        
        gripper_goal.trajectory.points = [point]
        gripper_goal.trajectory.header.stamp = rospy.Time.now()  # Start

        
        grip_client.send_goal(gripper_goal)
        rospy.sleep(1)                     # Wait for the gripper action to be finish
        rospy.loginfo("Gripper: close")



    def arm_straight_down(self):
        current_state = self.group_arm.get_current_joint_values()
        initial_joint_group_positions = list(current_state)
        initial_joint_group_positions[:5] = [1.57, 1, 0, 1.2, -1.57] 
       

        self.round_configuration(initial_joint_group_positions)
        rospy.loginfo("Arm: straight down ")
        rospy.sleep(1)        
        
    def arm_straight_up(self):
        current_state = self.group_arm.get_current_joint_values()
        initial_joint_group_positions = list(current_state)
        initial_joint_group_positions[:5] = [1.57, 1, 0, 0.5, -1.57] 
       

        self.round_configuration(initial_joint_group_positions)
        rospy.loginfo("Arm: straight up")
        rospy.sleep(1)
        
        
    def arm_up(self):
        current_state = self.group_arm.get_current_joint_values()
        initial_joint_group_positions = list(current_state)
        initial_joint_group_positions[:5] =  [0.2, 1,  1.5, 1.1, -2]
       

        self.round_configuration(initial_joint_group_positions)
        rospy.loginfo("Arm: up")
        rospy.sleep(1)
        
        
    def assign_initial_configuration(self):
        feedback_status = "Assign initial configuration"
        rospy.loginfo(feedback_status)

        current_state = self.group_arm.get_current_joint_values()
        initial_joint_group_positions = list(current_state)
        initial_joint_group_positions[:6] = [0.2, -1.3, -0.2, 1.9, -1.5, 1.3]
        
        joint_names = self.group_arm.get_joints()

        # Appliquer un arrondi ou ajustement si nécessaire 
        self.round_configuration(initial_joint_group_positions)

        rospy.sleep(1)                     # Wait for the arm action to be finish
        rospy.loginfo("Arm: config init")
        
    def gripper_down(self):
        current_state = self.group_arm.get_current_joint_values()
        initial_joint_group_positions = list(current_state)
        initial_joint_group_positions[5] = 1.3
       
        self.round_configuration(initial_joint_group_positions)
        rospy.loginfo("Gripper: down")
        rospy.sleep(1)
        
    def gripper_up(self):
        current_state = self.group_arm.get_current_joint_values()
        initial_joint_group_positions = list(current_state)
        initial_joint_group_positions[5] = 0
       
        self.round_configuration(initial_joint_group_positions)
        rospy.loginfo("Gripper: up")
        rospy.sleep(1)
        


    def round_configuration(self, joint_positions):
        """
        Arrondit ou ajuste les positions des articulations pour des configurations valides.

        :param joint_positions: Liste des positions des articulations.
        """
        for i in range(len(joint_positions)):
            joint_positions[i] = round(joint_positions[i], 2)  # Ajuste à deux décimales

        # Appliquer les nouvelles positions
        self.group_arm.set_joint_value_target(joint_positions)
        self.group_arm.go(wait=True)  # Effectuer le mouvement


class CoeffRequester:
    def __init__(self):
        pass

    def request_coefficients(self):
        rospy.wait_for_service('/get_coefficients')

        try:
            coeff_service = rospy.ServiceProxy('/get_coefficients', Trigger)
            response = coeff_service()

            if response.success:
                m, q = map(float, response.message.split(','))
                rospy.loginfo(f"Received coefficients: m = {m}, q = {q}")
                return m, q
            else:
                rospy.logwarn("Failed to retrieve coefficients.")
                return None, None
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None, None





if __name__ == '__main__':
    try:
        global stop_publishing
        
        # Initialize 
        rospy.init_node('global_navigation_node', anonymous=True)
        navigator = Navigator()	#Move the robotin the room    
        armManipulator = ArmManipulator()
        
        #--- Start
        #armManipulator.assign_initial_configuration()      #Init the arm        
        armManipulator.arm_up()
        navigator.send_goal({'x': 8.75, 'y': 0.129, 'yaw': 4.81})#Go to the main room
        

        #--- Move to scan all the tags
        # 2nd table (Search for tag 10)
        navigator.send_goal({'x': 8.95, 'y': -2,   'yaw': 3.14}) # Acces to the table
        navigator.send_goal({'x': 8.6,  'y': -1.8, 'yaw': 3.14}) # Goes to the table 
        navigator.send_goal({'x': 8.6,  'y': -1.8, 'yaw': 2.31}) # Look from right to left
        navigator.send_goal({'x': 8.6,  'y': -1.8, 'yaw': 4.97})
        
        
        # 1st table
        #front
        navigator.send_goal({'x': 8.95, 'y': -3,   'yaw': 3.14}) # Acces to front the table
        navigator.send_goal({'x': 8.6,  'y': -3,   'yaw': 3.14}) # Goes to the table 
        navigator.send_goal({'x': 8.6,  'y': -3,   'yaw': 2.31}) # Look from right to left
        navigator.send_goal({'x': 8.6,  'y': -3,   'yaw': 3.97})

        
        """
        # It take too much time to scan the 3 sides, and we have the 3 objects on the 1st side
        
        #left
        navigator.send_goal({'x': 8.95, 'y': -4,   'yaw': 4.81}) # Move around the table
        navigator.send_goal({'x': 7.8,  'y': -4,   'yaw': 1.67}) # Acces to side the table
        navigator.send_goal({'x': 7.8,  'y': -3.8, 'yaw': 1.67}) # Goes to the table
        navigator.send_goal({'x': 7.8,  'y': -3.8, 'yaw': 0.84}) # Look from right to left
        navigator.send_goal({'x': 7.8,  'y': -3.8, 'yaw': 2.5 })
        #back
        navigator.send_goal({'x': 6.7,  'y': -4,   'yaw': 1.67 }) # Move around the table    
        navigator.send_goal({'x': 6.7,  'y': -2.9, 'yaw': 0  }) # Acces to back the table
        navigator.send_goal({'x': 6.8,    'y': -2.9, 'yaw': 0   }) # Goes to the table 
        navigator.send_goal({'x': 6.8,    'y': -2.9, 'yaw': 0.83}) # Look from right to left
        navigator.send_goal({'x': 6.8,    'y': -2.9, 'yaw': 5.64})
        """
        
        
        """
        # It don't work, we define the objects localisation later
        
        #--- Request AprilTag data and select 3 objects
        requester = AprilTagsRequester()
        tags = requester.request_april_tags()
        waypoints_pick = []

        if tags:
            # Select the top 3 tags based on X coordinate
            waypoints_pick = navigator.select_and_generate_waypoints(tags)
        """

        #--- Get the line coeff and the position to place the object
        offset_x = 0.82+0.014
        offset_y = 0.1 +0.0066
        """
        m=0
        q=0
        try:
            coeff_requester = CoeffRequester()
            m, q = coeff_requester.request_coefficients()
            if m is not None and q is not None:
                rospy.loginfo(f"Successfully retrieved coefficients: m = {m}, q = {q}")
            else:
                rospy.logerr("Unable to retrieve coefficients.")
        except rospy.ROSInterruptException:
            rospy.loginfo("Node interrupted.")
        """
        
        """
        #The coefficient arn't good, we define the placeing localisation later
        
        waypoints_place = []
        j=0
        for xi in x:
            yi = m * xi + q
            waypoint = {'x': xi+arm_length, 'y': yi, 'yaw': yaw}
            waypoints_place.append(waypoint)
            rospy.loginfo(f"Object {j}: {waypoints_pick[j]} to {waypoint}")
            j=j+1
        

        #--- Pick and place
        navigator.send_goal({'x': 6.8,  'y': -4,   'yaw': 4.81})  
        navigator.send_goal({'x': 8.95, 'y': -4,   'yaw': 3.14}) 
        navigator.send_goal({'x': 8.95, 'y': 0.129, 'yaw': 0.0})
        """
        
        #Objects localisaiton found by node_b.py
        waypoints_pick = []
        #Object 3 (rouge loin)
        waypoints_pick.append({'x':7.7393+offset_x,'y':-3.3519+offset_y,'yaw':3.14})
        #Object 5 (rouge proche)
        waypoints_pick.append({'x':8.0042+offset_x,'y':-2.8155+offset_y,'yaw':3.14})
        #Object 7 (vert)
        waypoints_pick.append({'x':8.0827+offset_x,'y':-3.0355+offset_y,'yaw':3.14})
        #Object 8 (bleu)
        waypoints_pick.append({'x':8.0739+offset_x,'y':-3.1817+offset_y,'yaw':3.14})
        
        
        
        #Placing localisation
        waypoints_place = []
        waypoints_place.append({'x':8.1+offset_x,'y':-2.2+offset_y,'yaw':3.14})
        waypoints_place.append({'x':8+offset_x,'y':-2.1+offset_y,  'yaw':3.14})
        waypoints_place.append({'x':7.9+offset_x,'y':-2+offset_y,  'yaw':3.14})
        waypoints_place.append({'x':7.8+offset_x,'y':-1.9+offset_y,'yaw':3.14})
        


        #Pick and place 3 times
        for i in range (4):
            armManipulator.arm_up()
            armManipulator.gripper_down()
            armManipulator.gripper_open()
            #Go to the object and pick it
            navigator.send_goal({'x':8.95,'y':-3.5,'yaw':1.67})
            navigator.send_goal(waypoints_pick[i])
            armManipulator.gripper_down()
            armManipulator.arm_straight_up()
            armManipulator.arm_straight_down()
            armManipulator.gripper_down()
            armManipulator.arm_straight_down()
            armManipulator.gripper_close()
            armManipulator.arm_straight_up()
            armManipulator.arm_up()
            #Go to the 2nd table and place it
            navigator.send_goal(waypoints_place[i])
            armManipulator.arm_straight_up()
            armManipulator.arm_straight_down()
            armManipulator.gripper_open()
            armManipulator.arm_straight_up()
            armManipulator.arm_up()

        #End
        navigator.send_goal({'x': 8.95, 'y': -2.5,   'yaw': 4.81}) 
        armManipulator.assign_initial_configuration()      #Init the arm


   
        
        
        
        
        
        
        


    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
        
        
        
        
        
        
        
        
        
        
        

