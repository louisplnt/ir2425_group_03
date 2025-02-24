#!/usr/bin/env python3


import rospy, actionlib, math
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, PoseStamped
from gazebo_msgs.srv import GetModelState
from moveit_commander import PlanningSceneInterface, SolidPrimitive, CollisionObject


class tiago_navigation:

    def __init__(self):
        
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.cmd_vel_pub = rospy.Publisher("mobile_base_controller/cmd_vel", Twist, queue_size=10)


    def go_to_point(self, x, y, angle_degrees):
        
        angle_radians = math.radians(angle_degrees)
        quaternion = quaternion_from_euler(0, 0, angle_radians)

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        rospy.loginfo(f"Sending goal to position: x = {x}, y = {y}, angle = {angle_degrees}°")
        self.client.send_goal(goal)
        self.client.wait_for_result()


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
        pose_stamped.pose = pose
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


def callback(msg):

    rospy.loginfo(f"{msg.data}")


if __name__ == "__main__":

    rospy.init_node("node_c")

    rospy.Subscriber("coeffs_topic", String, callback)

    #navigator = tiago_navigation()
    #navigator.move_straight(8, 0.5)

    collision = collision_manager()

    pose = collision.get_model_position("pick_table_clone")
    collision.add_table_collision("pick_table_clone", pose)

    pose = collision.get_model_position("pick_table")
    collision.add_table_collision("pick_table", pose)

    pose = collision.get_model_position("Triangle_9")
    collision.add_object_collision(9, pose)
    collision.remove_object_collision(9)

    pose = collision.get_model_position("cube_6")
    collision.add_object_collision(6, pose)
    collision.remove_object_collision(6)

    pose = collision.get_model_position("Hexagon_2")
    collision.add_object_collision(2, pose)
    collision.remove_object_collision(2)

    rospy.spin()