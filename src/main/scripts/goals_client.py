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
	
	points_x = [1.13, 1.19, 0.311, 0.327, -1.37]
	points_y = [-1.64, -0.116, 1.29, 2.26, -0.0822]
	
	for i in range(0,5):
		
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
