#!/usr/bin/env python3

import rospy
import actionlib
import moveit_commander
import sys
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, SolidPrimitive, CollisionObject
from std_srvs.srv import Trigger
from gazebo_msgs.srv import GetModelState
from copy import deepcopy



class Navigator:
    def __init__(self):
        rospy.init_node('global_navigation_node', anonymous=True)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.stop_navigation = False  # Flag to indicate if navigation should stop
        self.cmd_vel_pub = rospy.Publisher("mobile_base_controller/cmd_vel", Twist, queue_size=10)


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

    
    def move_straight(self, distance, speed=0.2):

        twist = Twist()
        twist.linear.x = speed
        rate = rospy.Rate(10)
        duration = abs(distance / speed)
        start_time = rospy.Time.now().to_sec()

        rospy.loginfo("Moving straight")
        while rospy.Time.now().to_sec() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        twist.linear.x = 0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Movement completed")

            


class ArmManipulator:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander("arm_torso")
        #self.gripper_group = MoveGroupCommander("gripper")

        # Set planning parameters
        self.arm_group.set_max_velocity_scaling_factor(0.5)
        self.arm_group.set_max_acceleration_scaling_factor(0.5)
        self.arm_group.set_planning_time(10.0)

    def move_to_position(self, x, y, z):
        """
        Move the arm to a specific position with a downward orientation.
        """
        rospy.loginfo(f"Move the arm to: {x}, {y}, {z}")
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "base_link"
        
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z

        quaternion = quaternion_from_euler(0, 1.5707, 0)
        goal_pose.pose.orientation.x = quaternion[0]
        goal_pose.pose.orientation.y = quaternion[1]
        goal_pose.pose.orientation.z = quaternion[2]
        goal_pose.pose.orientation.w = quaternion[3]

        self.arm_group.set_pose_target(goal_pose) 
        
        """
        # Increase planning time and allow replanning
        self.arm_group.set_planning_time(15.0)  # Increased from 10 to 15 seconds
        self.arm_group.set_num_planning_attempts(10)  # Try multiple times
        self.arm_group.allow_replanning(True)  # Allow replanning on failure
    
        # Set tolerances and planner
        self.arm_group.set_goal_tolerance(0.02)  # 2 cm tolerance
        self.arm_group.set_goal_orientation_tolerance(0.1)  # Orientation tolerance
        self.arm_group.set_planner_id("RRTConnectkConfigDefault")  # Faster planner
        """
        
        
        # Check if planning was successful
        success = self.arm_group.go(wait=True)
        
        # Stop the arm and clear targets
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        if success:
            rospy.loginfo(f"Arm moved to position: x={x}, y={y}, z={z}")
        else:
            rospy.logerr("Failed to move arm to position.")
    
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

    def attach_object(self, object_name):
        """
        Attach an object to the gripper for manipulation.
        """
        self.scene.attach_box("arm_7_link", object_name)
        rospy.loginfo(f"Attached {object_name} to gripper.")

    def detach_object(self, object_name):
        """
        Detach the object from the gripper.
        """
        self.scene.remove_attached_object("arm_7_link", object_name)
        rospy.loginfo(f"Detached {object_name} from gripper.")
        
    def arm_safe_position(self):
        """
        Move the arm to a safe position, close to the robot's body.
        """
        safe_configuration = {
            'torso_lift_joint': 0.35,
            'arm_1_joint': 0.2,
            'arm_2_joint': 1,
            'arm_3_joint': 1.5,
            'arm_4_joint': 1.2,
            'arm_5_joint': 2,
            'arm_6_joint': 1.3,
            'arm_7_joint': 0
        }

        try:
            self.arm_group.set_joint_value_target(safe_configuration)
            self.arm_group.go(wait=True)
            self.arm_group.stop()
            rospy.loginfo("Arm moved to safe position.")
        except Exception as e:
            rospy.logerr(f"Failed to move arm to safe position: {e}")
    
    def arm_up(self):
        safe_configuration = {
            'torso_lift_joint': 0.35,
            'arm_1_joint': 1.57,
            'arm_2_joint': 1,
            'arm_3_joint': 0,
            'arm_4_joint': 0.5,
            'arm_5_joint': -1.57,
            'arm_6_joint': 1.3,
            'arm_7_joint': 0
        }

        try:
            self.arm_group.set_joint_value_target(safe_configuration)
            self.arm_group.go(wait=True)
            self.arm_group.stop()
            rospy.loginfo("Arm moved to up position.")
        except Exception as e:
            rospy.logerr(f"Failed to move arm to up position: {e}")



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


class AprilTagsRequester:
    def __init__(self):
        self.list_apriltags =  [[] for _ in range(11)]

    def request_april_tags(self):
        """
        Calls the `/ping_service` to request a ping response from node_b_main.
        """
        rospy.loginfo("AprilTagsRequester: AprilTags requested to node_b:")
        rospy.wait_for_service('/get_apriltags_detected', timeout=5.0)
        try:
            ping_service = rospy.ServiceProxy('/get_apriltags_detected', Trigger)
            response = ping_service()

            if response.success:
                rospy.loginfo("AprilTagsRequester: AprilTags provided by node_b:")

                # Split the apriltags information separated by ';'
                tags = response.message.split(';')

                for tag in tags:
                    # Split each information separated by ','
                    data = tag.split(',')
                    id_tag = int(data[0])  # ID as integer
                    x = float(data[1])     # x as float
                    y = float(data[2])     # y as float
                    z = float(data[3])     # z as float

                    rospy.loginfo(f"- apriltag_{id_tag}:  {x}, {y}, {z}")
                    self.list_apriltags[id_tag] =[x, y, z]

            return self.list_apriltags

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return []
        

class collision_manager:
    
    def __init__(self):

        rospy.wait_for_service("/gazebo/get_model_state")
        self.get_model_state_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        self.scene = PlanningSceneInterface()
        rospy.sleep(1)

        self.object_states = {i: 0 for i in range (1, 10)}
        self.collision_object_pub = rospy.Publisher("/collision_object", CollisionObject, queue_size=10)


    def get_model_position(self, object_name, reference_frame = "world"):
        
        try:
            response = self.get_model_state_srv(object_name, reference_frame)
            if response.success:
                return response.pose
            else:
                rospy.logwarn(f"Impossible de récupérer la position du modèle '{object_name}'")
                return None
        
        except rospy.ServiceException as e:
            rospy.logerr(f"Erreur lors de l'appel du service : {e}")
            return None
        
        
    def add_table_collision(self, object_name, pose, reference_frame = "base_link"):

        size = (0.92, 0.92, 0.92)
        
        square_pose = PoseStamped()
        square_pose.header.frame_id = reference_frame
        square_pose.pose = pose

        square_pose.pose.position.z += size[2] / 2

        self.scene.attach_box(reference_frame, object_name, square_pose, size)
        rospy.loginfo(f"Collision carrée '{object_name}' attachée au frame '{reference_frame}'")


    def add_object_collision(self, apriltag_id, pose, reference_frame="base_link"):

        if self.object_states.get(apriltag_id, 0) == 1:
            rospy.logwarn(f"L'objet avec l'id {apriltag_id} existe déjà. Collision non ajoutée")
            return

        if apriltag_id in [1, 2, 3]:
            object_type = "hexagonal_prism"
            size = (0.05, 0.05, 0.1)

        elif apriltag_id in [4, 5, 6]:
            object_type = "cube"
            size = (0.05, 0.05, 0.05)

        elif apriltag_id in [7, 8, 9]:
            object_type = "triangular_prism"
            size = (0.07, 0.035, 0.05)

        collision_object = CollisionObject()
        collision_object.header.frame_id = reference_frame
        collision_object.id = f"object_{apriltag_id}"

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = reference_frame
        pose_stamped.pose = deepcopy(pose.pose)
        pose_stamped.pose.position.z += size[2] / 2

        primitive = SolidPrimitive()
        
        if object_type == "hexagonal_prism":
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [size[2], size[0]]
            rospy.loginfo(f"Collision de type 'prisme hexagonal' ajoutée pour l'objet {apriltag_id}")

        elif object_type == "cube":
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [size[0], size[1], size[2]]
            rospy.loginfo(f"Collision de type 'cube' ajoutée pour l'objet {apriltag_id}")

        elif object_type == "triangular_prism":
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [size[0], size[1], size[2]]
            rospy.loginfo(f"Collision de type 'prisme triangulaire' ajoutée pour l'objet {apriltag_id}")

        else:
            rospy.logwarn(f"Type d'objet '{object_type}' non pris en charge pour l'ID {apriltag_id}.")
            return

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose_stamped.pose)
        collision_object.operation = CollisionObject.ADD

        self.scene.add_object(collision_object)

        self.collision_object_pub.publish(collision_object)

        self.object_states[apriltag_id] = 1


    def remove_object_collision(self, apriltag_id):

        if self.object_states.get(apriltag_id, 0) == 1:
            object_id = f"object_{apriltag_id}"
            self.scene.remove_world_object(object_id)
            self.object_states[apriltag_id] = 0
            rospy.loginfo(f"Collision de l'objet {apriltag_id} supprimée")

        else:
            rospy.logwarn(f"Aucune collision trouvée pour l'id {apriltag_id}")


if __name__ == '__main__':
    try:
        global stop_publishing

        #---- 0 : INIT -----
        rospy.init_node('global_navigation_node', anonymous=True)

        # Move and rotate the robot in the room
        navigator = Navigator()
        #Go to the main room
        navigator.move_straight(8, 0.5)
        # Move the arm, torso and gripper
        armManipulator = ArmManipulator()
        # Will request the list of detected AprilTags and their positions
        apriltags_requester = AprilTagsRequester()
        # Will request the m and q coefficient
        coeff_requester = CoeffRequester()

        armManipulator.arm_safe_position()

        #Initialize collision class and create pick & placing tables collisions
        collision = collision_manager()
        pose = collision.get_model_position("pick_table_clone")
        collision.add_table_collision("pick_table_clone", pose)
        pose = collision.get_model_position("pick_table")
        collision.add_table_collision("pick_table", pose) 


        # ----- 1 : MOVE AROUND TO SCAN -----
        # Go in the room
        #navigator.move_straight(8, 0.2)
        #navigator.send_goal({'x': 8.75, 'y': 0.129, 'yaw': 4.81})  # Go to the main room

        # Placing table (Search for tag 10)
        navigator.send_goal({'x': 9, 'y': -2, 'yaw': 3.14})  # Access to the table
        navigator.send_goal({'x': 8.6, 'y': -1.8, 'yaw': 3.14})  # Goes to the table
        navigator.send_goal({'x': 8.6, 'y': -1.8, 'yaw': 2.31})  # Look from right to left
        navigator.send_goal({'x': 8.6, 'y': -1.8, 'yaw': 4.97})

        # Picking table
        # front
        navigator.send_goal({'x': 9, 'y': -3, 'yaw': 3.14})  # Access to front the table
        navigator.send_goal({'x': 8.6, 'y': -3, 'yaw': 3.14})  # Goes to the table
        navigator.send_goal({'x': 8.6, 'y': -3, 'yaw': 2.31})  # Look from right to left
        navigator.send_goal({'x': 8.6, 'y': -3, 'yaw': 3.97})

        """
        # left
        navigator.send_goal({'x': 8.95, 'y': -4, 'yaw': 4.81})  # Move around the table
        navigator.send_goal({'x': 7.8, 'y': -4, 'yaw': 1.67})  # Access to side the table
        navigator.send_goal({'x': 7.8, 'y': -3.8, 'yaw': 1.67})  # Goes to the table
        navigator.send_goal({'x': 7.8, 'y': -3.8, 'yaw': 0.84})  # Look from right to left
        navigator.send_goal({'x': 7.8, 'y': -3.8, 'yaw': 2.5})
        
        # It take too much time to scan the 3 sides (my real time factor is 0.05), and we have enough object already
        #back
        navigator.send_goal({'x': 6.7,  'y': -4,   'yaw': 1.67 }) # Move around the table    
        navigator.send_goal({'x': 6.7,  'y': -2.9, 'yaw': 0  }) # Acces to back the table
        navigator.send_goal({'x': 6.8,    'y': -2.9, 'yaw': 0   }) # Goes to the table 
        navigator.send_goal({'x': 6.8,    'y': -2.9, 'yaw': 0.83}) # Look from right to left
        navigator.send_goal({'x': 6.8,    'y': -2.9, 'yaw': 5.64})
        
        navigator.send_goal({'x': 6.7,  'y': -4,   'yaw': 1.67 }) # Move around the table    
        """
        #navigator.send_goal({'x': 8.95, 'y': -4, 'yaw': 4.81})  # Move around the table
        
        
       

        # ----- 2 : GET THE (m,q) COEFFICIENTS -----
        m, q = coeff_requester.request_coefficients()
        if m is not None and q is not None:
            rospy.loginfo(f"Successfully retrieved coefficients: m = {m}, q = {q}")
        else:
            rospy.logerr("Unable to retrieve coefficients.")


        # ----- 3 : GET THE LIST OF THE APRILTAGS -----
        apriltags_requester.request_april_tags()

        #Create collisions for each object
        for apriltag_id in range(1, 10):  # IDs des objets
            if apriltags_requester.list_apriltags[apriltag_id]:
                pose = PoseStamped()
                pose.pose.position.x = apriltags_requester.list_apriltags[apriltag_id][0]
                pose.pose.position.y = apriltags_requester.list_apriltags[apriltag_id][1]
                pose.pose.position.z = apriltags_requester.list_apriltags[apriltag_id][2]
                collision.add_object_collision(apriltag_id, pose)
        

        # Calculate distances and store them in a list
        docking_x = 8.7
        docking_y = -3
        list_distance_dockingpoint = []
        for i in range(10):
            if not apriltags_requester.list_apriltags[i]:
                list_distance_dockingpoint.append(100)  # Distance should be very high if None
            else:
                list_distance_dockingpoint.append((apriltags_requester.list_apriltags[i][0] - docking_x) ** 2 +
                                                  (apriltags_requester.list_apriltags[i][1] - docking_y) ** 2)
                                                  
                                
        # Find the 3 smallest distances and get their IDs
        closest_ids = sorted(range(len(list_distance_dockingpoint)), key=lambda k: list_distance_dockingpoint[k])[:3]
        rospy.loginfo(f"Closest IDs to docking point: {closest_ids}")                   
        
        
        #----- 4 : GET THE PICKING AND PLACING LOCALISATION
        

        waypoints_pick = []
        for i in range(3):     
            waypoints_pick.append(apriltags_requester.list_apriltags[closest_ids[i]])
            
            z_pick = 0
            #Triangular prism
            if closest_ids[i] in [7,8,9]: 
                #Gipper length - prism offset + object z position 
                z_pick = 0.226 - 0.019 + waypoints_pick[i][2]
            #Cube
            elif closest_ids[i] in [4,5,6]: 
                #Gipper length - object height/2 + object z position 
                z_pick = 0.226 - 0.05/2 + waypoints_pick[i][2]
            #Hexagonal prism
            else:
                #Gipper length - object height/2 + object z position 
                z_pick = 0.226 - 0.1/2 + waypoints_pick[i][2]
                
            waypoints_pick[i][2]  = z_pick 
                
            
         
                
        waypoints_place = []
        x_tag10 = apriltags_requester.list_apriltags[10][0]
        y_tag10 = apriltags_requester.list_apriltags[10][1]
        
        for i in range(3):
            delta_x = 0.1*(1+i)
            x_new = x_tag10 - delta_x 
            y_new = y_tag10 + (m * delta_x + q)*0.1
            
            waypoints_place.append([x_new, y_new, 0.9])

      
      
      
        #----- 5 : PICK AND PLACE 3 OBJECTS -----

        navigator.send_goal({'x': 8.95, 'y': -3, 'yaw': 3.14})  # Access to front the table
        armManipulator.arm_up()
     
        #Pick and place 3 times
        for i in range (3):

            # Go in front of the picking table
            navigator.send_goal({'x': 8.7, 'y': -3, 'yaw': 3.14})    
            
            # Put the arm above the object, at the middle of the height and pick it
            pick_pos = waypoints_pick[i]
            armManipulator.move_to_position(pick_pos[0], pick_pos[1], pick_pos[2] + 0.2)
            armManipulator.gripper_open()

            #Delete collision of object i
            collision.remove_object_collision(closest_ids[i])

            armManipulator.move_to_position(pick_pos[0], pick_pos[1], pick_pos[2])
            armManipulator.gripper_close()
            armManipulator.attach_object("picked_object")
            armManipulator.move_to_position(pick_pos[0], pick_pos[1], pick_pos[2] + 0.2)
            
            # Go to the placing table
            navigator.send_goal({'x': 8.7, 'y': -1.8, 'yaw': 3.14})  # Table place
            
            # Place the object
            place_pos = waypoints_place[i]
            armManipulator.move_to_position(place_pos[0], place_pos[1], place_pos[2] + 0.2)
            armManipulator.move_to_position(place_pos[0], place_pos[1], place_pos[2])
            armManipulator.gripper_open()
            armManipulator.detach_object("picked_object")

            #Recreate collision on the placing table
            pose.pose.position.x = place_pos[0]
            pose.pose.position.y = place_pos[1]
            pose.pose.position.z = place_pos[2]
            collision.add_object_collision(closest_ids[i], pose)

            armManipulator.move_to_position(place_pos[0], place_pos[1], place_pos[2] + 0.2)
        
        

        #End
        navigator.send_goal({'x': 8.95, 'y': -2.5,   'yaw': 4.81}) 






        # Stop navigation if requested by status callback
        if navigator.stop_navigation:
            rospy.loginfo("Stopping navigation")
            rospy.signal_shutdown("Research completed")

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception occurred.")

