#!/usr/bin/python3

import sys
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped


def move_to_goals():

	rospy.init_node("goals_client")

	client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	client.wait_for_server()

	points_x = [-0.3,  2.25,  3.65, 2.55, 1.12, 1.9, -0.77, -1.25,  -1.4, 0]
	points_y = [-1.6, -0.7, -1.23, 1.69, -0.3, 2.1,  1.2,   1.6,   -0.1, 0]

	for i in range(0,len(points_x)):

		goal = MoveBaseGoal()

		# set next point as goal
		goal.target_pose.pose.position.x = points_x[i]
		goal.target_pose.pose.position.y = points_y[i]
		goal.target_pose.pose.orientation.w = 1

		goal.target_pose.header.frame_id = "map"

		print("Sending goal %d" % i)

		client.send_goal(goal)

		while client.get_state() < 3:
			print("moving ...");
			client.wait_for_result(rospy.Duration(1))

		if client.get_state() == 3:
			print("Goal %d reached" % i)
		else:
			print("Unable to reach goal %d, moving on" % i)

	print("Done")

if __name__ == "__main__":
	move_to_goals()
