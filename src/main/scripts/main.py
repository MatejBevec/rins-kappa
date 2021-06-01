#!/usr/bin/python3

import sys
import math
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point

# IMPORTED CLASSES (SOME MIGHT NEED TO GO INTO SEPARATE NODES)

from cylinder_filter import CylinderFilter

#from aproaching_class_old import Approacher
from aproachingclass import Approacher
from extend_retract_arm import Arm_Mover

# move to main if there is time
from extract_qr import QRExtractor


# QUESTIONS:
# How to turn off nodes after exploration?

# Is blocking functional good enough or do we need action server-client setup?
# Recovery procedure needed!


class Agent():

	def __init__(self):

		# states
		self.INIT = 0
		self.EXPLORE = 1
		self.ASK_FACE = 2

		self.state = 0

		#other
		rospy.init_node("goals_mb_client")
		self.mb_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		print("Waiting for move-base")
		self.mb_client.wait_for_server()

		self.preset_goals = [
			(-0.3, -1,6),
			(2.25, -0.7),
			(3.65, -1.23),
			(2.55, 1.69),
			(1.12, -0.3),
			(1.9, 2.1),
			(-0.77, 1.2),
			(-1.25, 1.6),
			(-1.4, 0),
			(0,0)
			]

		self.cylinders = {} # "color": position
		self.rings = {} # "color": position
		self.faces = [] # faces[i]["position"], faces[i]["img"]

		self.test_cylinders = {}
		p = Point()
		p.x, p.y = -0.68, -2
		self.test_cylinders["blue"] = p
		p = Point()
		p.x, p.y = 2.95, 0.4
		self.test_cylinders["green"] = p
		p = Point()
		p.x, p.y = 2.5, 2.6
		self.test_cylinders["yellow"] = p
		p = Point()
		p.x, p.y = -1.8, -0.12
		self.test_cylinders["red"] = p

		self.test_rings = {
			"green": Point(-1,0.67, 1),
			"black": Point(-0.15, 1.5, 1),
			"red": Point(2.3, 1.3, 1),
			"blue": Point(2.8, -1.6, 1)
		}

		self.test_faces = [
			{"position": Point(1.35, 2.7, 0.5), "name": "bluemask"},
			{"position": Point(0.74, 0.74, 0.5), "name": "gargamel"},
			{"position": Point(4.1, -1, 0.5), "name": "greenmask"},
			{"position": Point(0.43, -0.26, 0.5), "name": "nomask_girl"}
		]

		self.SAFE_DIST = 2 # safe distance for social distancing
		self.unsafe_pairs = [] #(face_i, face_j) pairs of faces that are too close

		self.current_face = 0 # which face to greet next
		#self.faces_info = [None for face in self.faces] # information about faces from QR codes
		#self.faces_info_guess = [] # info about faces from digits, conversation ...


	def change_state_to(self, new_state):
		# switches state and performs necessary operation for transition
		if self.state == self.INIT and new_state == self.EXPLORE:
			# only makes sense without blocking functions
			pass
			# [todo]: initialize and start exploration loop
		if self.state == self.EXPLORE and new_state == self.ASK_FACE:
			pass
			# [todo]: disable detection nodes, start approaching


		self.state = new_state

	def distance(self, x1, y1, x2, y2):
		return math.hypot(x1-x2, y1-y2)

	def get_unsafe_pairs(self, appr):
		if len(self.faces) == 0:
			print("No faces detected yet.")
			return

		num_points = 5 #for checking wall between
		unsafe_pairs = []
		for i in range(0, len(self.faces)):
			for j in range(i+1, len(self.faces)):
				pos_a = self.faces[i]["position"]
				pos_b = self.faces[j]["position"]
				dist = self.distance(pos_a.x, pos_a.y, pos_b.x, pos_b.y)
				wall_between = not appr.checkwall(pos_a.x, pos_a.y, pos_b.x, pos_b.y, num_points)
				print(f"Distance between ({i}) and ({j}) is {dist}, wall: {wall_between}")

				if dist < self.SAFE_DIST:
					unsafe_pairs.append((i, j, dist, wall_between))

		self.unsafe_pairs = unsafe_pairs
		return unsafe_pairs

	def face_pair_midpoint(self, i, j):
		pos_a = self.faces[i]["position"]
		pos_b = self.faces[j]["position"]
		mid_x = (pos_a.x + pos_b.x) / 2
		mid_y = (pos_a.y + pos_b.y) / 2

		return mid_x, mid_y

	def print_message(self, color, string):
		ansi = {
			"red": "\u001b[31m",
			"green": "\u001b[32m",
			"yellow": "\u001b[33m",
			"blue": "\u001b[34m",
			"black": "\u001b[30m",
			"white": "\u001b[37m"
			}
		print(">>>>>\n")
		print(f"{ansi[color]} {string} \u001b[37m")
		print("\n>>>>>")


	def move_base_to(self, x, y):
		# send goal to move base and wait for arrival (blocking!)
		# blocks and return True on reached goal and False on failure

		goal = MoveBaseGoal()
		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y
		goal.target_pose.pose.orientation.w = 1

		goal.target_pose.header.frame_id = "map"

		print("Sending move_base to (%.2f, %.2f)" % (x, y))

		self.mb_client.send_goal(goal)

		while self.mb_client.get_state() < 3:
			print("moving ...");
			self.mb_client.wait_for_result(rospy.Duration(1))

		if self.mb_client.get_state() == 3:
			print("Goal reached")
		else:
			print("Unable to reach goal , moving on")

	def explore_goals(self, goals):
		print("Starting exploration with goals:")
		print(goals)

		for pos in goals:
			x = pos[0]
			y = pos[1]
			self.move_base_to(x,y)

		print("Finished moving")

	def populate_detections(self):
		# Get finalized detections from filter classes and populate arrays
		print("population detections")
		pass

	# "STEP" FUNCTIONS - either this or state mgmt, we'll see
	# ---------------------------------------------------------

	def explore_step(self):
		cylinder_f = CylinderFilter()
		#cylinder_f.spin()
		self.explore_goals(self.preset_goals)
		#self.populate_detections()
		self.cylinders = cylinder_f.get_final_detections()
		print(self.cylinders)

	def warning_step(self, appr):
		self.get_unsafe_pairs(appr)

		for pair in self.unsafe_pairs:
			i,j = pair[0], pair[1]
			name_i, name_j = self.faces[i]["name"], self.faces[j]["name"]
			print(f"{name_i} and {name_j} are too close ({pair[2]:.1f}m).")

			wall_between = pair[3]
			if wall_between:
				self.print_message("yellow", f"{name_i} AND {name_j} ARE SAFE - THERE IS A WALL BETWEEN THEM")
				continue

			print(f"Going to warn {name_i} and {name_j}")
			mid_x, mid_y = self.face_pair_midpoint(i,j)
			appr.approachnew(mid_x, mid_y, "cylinder")

			self.print_message("yellow", f"WARNING FOR {name_i} AND {name_j}, PLEASE RESPECT SOCIAL DISTANCE!")
			#appr.doMoveBack("cylinder")

	def test_approach_step(self, appr, arm_mover):

		for color in self.cylinders:
			print("approaching " + color + " cylinder")
			pos = self.cylinders[color]
			appr.approachnew(pos.x, pos.y, "cylinder")
			arm_mover.extend_retract()
			appr.moveBackType("cylinder")


	def approach_one_face(self, i, appr, arm_mover, qr_extr):

		face = self.faces[i]
		print("Approaching " + face["name"] + ".")
		pos = face["position"]
		appr.approachnew(pos.x, pos.y, "cylinder")
		appr.leftRight(20)
		data = qr_extr.getLastDetected()
		self.faces[i]["info"] = data

		self.print_message("yellow", f"HELLO, {face['name']}")
		print("Info about face:")
		print(data)

		arm_mover.extend_retract()
		appr.moveBackType("cylinder")

	def approach_one_cylinder(self, color, appr, arm_mover, qr_extr):

		pos = self.cylinders[color]
		print(f"Approaching {color} doctor.")
		appr.approachnew(pos.x, pos.y, "cylinder")
		#appr.leftRight(20)
		#dataset = qr_extr.getLastDataset()
		self.print_message(color, f"HELLO, {color} doctor.")
		appr.moveBackType("cylinder")

	def approach_one_ring(self, color, appr, arm_mover):

		pos = self.rings[color]
		print(f"Approaching {color} vaccine.")
		appr.approachnew(pos.x, pos.y, "cylinder")
		self.print_message("yellow", f"PICKING UP {color} vaccine.")
		arm_mover.extend_retract()


	# MAIN RUNTIME FUNCTION
	def runtime(self):
		self.faces = self.test_faces
		self.cylinders = self.test_cylinders
		self.rings = self.test_rings

		# SHADOWS OFF IN GAZEBO!

		appr = Approacher()
		arm_mover = Arm_Mover()
		qr_extr = QRExtractor()
		#qr_extr.visualize = True

		#self.test_approach_faces(appr, arm_mover, qr_extr)

		self.explore_step()
		self.warning_step(appr)

		for i in range(0, len(self.faces)):
			self.approach_one_face(i, appr, arm_mover, qr_extr)

			next_cyl = self.faces[i]["info"]["doctor"] if self.faces[i]["info"] else None
			if not next_cyl:
				print("Doctor not known, next face.")
				continue;

			self.approach_one_cylinder(next_cyl, appr, arm_mover, qr_extr)

			next_ring = self.faces[i]["info"]["vaccine"] if self.faces[i]["info"] else None
			if not next_ring:
				print("Vaccine not known, next face.")
				continue;

			self.approach_one_ring(next_ring, appr, arm_mover)


	def test(self):
		print("hello world")


if __name__ == "__main__":
	a = Agent()
	a.runtime()
