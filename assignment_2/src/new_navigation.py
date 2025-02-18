#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class TiagoNavigation:
    def __init__(self):
        rospy.init_node("tiago_navigation")
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("move_base action server prêt")

    def go_to(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = theta

        rospy.loginfo(f"Envoi du robot à la position: x = {x}, y = {y}, θ = {theta}")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo("Tiago est arrivé devant la table")

if __name__ == "__main__":
    navigation = TiagoNavigation()
    navigation.go_to(8, 0, 1.0)
    #navigation.go_to(, -0.553829, 1.0)

#test