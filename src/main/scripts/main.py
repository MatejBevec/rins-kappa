#!/usr/bin/python3

import sys
import math
import rospy
import actionlib
import os

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point

# IMPORTED CLASSES (SOME MIGHT NEED TO GO INTO SEPARATE NODES)

from cylinder_filter import CylinderFilter
from face_ring_wrapper import FaceRingWrapper

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
		rospy.init_node("goals_mb_client", log_level=rospy.WARN)

		self.mb_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		print("Waiting for move-base")
		self.mb_client.wait_for_server()

		self.preset_goals = [
			(-0.3, -1.6),
			(2.25, -0.7),
			(3.40, -1.23),
			(2.55, 1.60),
			(1.12, -0.3),
			(1.9, 2.1),
			(-0.77, 1.2),
			(-1.25, 1.6),
			(-1.4, 0),
			(-0.35, 0.71),
			(0, 0)
			]

		self.cylinders = {}  # "color": position
		self.rings = {}  # "color": position
		self.faces = []  # faces[i]["position"], faces[i]["img"]

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

		self.test_cylinders = {
			"blue": Point(0.44, 0.82, 0.5),
			"green": Point(-1.75, -0.29, 0.5),
			"red": Point(1.1, -0.78, 0.5),
			"yellow": Point(4, -1.3, 0.5)
		}

		self.test_rings = {
			"green": Point(-1.13, 0.71, 1),
			"black": Point(1.6, 1.4, 1),
			"red": Point(1.9, 1.7, 1),
			"blue": Point(2.9, -1.6, 1)
		}

		self.test_faces = [		
			{"position": Point(0.78, 0.61, 0.5), "name": "greenmask"},
			{"position": Point(1.55, 2.65, 0.5), "name": "bluemask"},
			{"position": Point(-0.32, -1.96, 0.5), "name": "nomask_girl"},
			{"position": Point(2.47, 2.6, 0.5), "name": "gargamel"}
		]

		self.SAFE_DIST = 2  # safe distance for social distancing
		self.unsafe_pairs = []  #(face_i, face_j) pairs of faces that are too close

		self.current_face = 0  # which face to greet next
		#self.faces_info = [None for face in self.faces] # information about faces from QR codes
		#self.faces_info_guess = [] # info about faces from digits, conversation ...

		# ----------- HELPER CLASSES: initialized in runtime() ------------


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
			"black": "\u001b[33m",  #30
			"white": "\u001b[37m"
			}
		print(">>>>>\n")
		print(f"{ansi[color]} {string} \u001b[37m")
		print("\n>>>>>")

	def kill_nodes(self, node_names):
		for name in node_names:
			os.system("rosnode kill " + name)

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

	def explore_step(self, cylinder_f, face_ring_f):
		self.explore_goals(self.preset_goals)

		self.cylinders = cylinder_f.get_final_detections()
		print("\nCylinders:\n", self.cylinders)

		self.faces = face_ring_f.get_final_face_detections()
		print("\nFaces:\n", self.faces)
		self.rings = face_ring_f.get_final_ring_detections()
		print("\nRings:\n", self.rings)
		# --> TODO: integrate

	def warning_step(self, appr):
		self.get_unsafe_pairs(appr)

		for pair in self.unsafe_pairs:
			i,j = pair[0], pair[1]
			#name_i, name_j = self.faces[i]["name"], self.faces[j]["name"]
			name_i, name_j = f"face {i}", f"face {j}"
			print(f"{name_i} and {name_j} are too close ({pair[2]:.1f}m).")

			if self.CHECK_WALL:
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
		if self.DETECT_DIGITS:
			qr_extr.detect_digits = True

		face = self.faces[i]
		#name = face["name"]
		name = f"face {i}"
		print("Approaching " + name + ".")
		pos = face["position"]
		appr.approachnew(pos.x, pos.y, "obraz")
		self.print_message("yellow", f"HELLO, {name}")
		appr.moveForward(0.1)
		appr.leftRight(25)
		data = qr_extr.getLastDetected()
		self.faces[i]["info"] = data

		print("Info about face:")
		print(data)

		# CHECK FOR DETECTED DIGITS AND MASK OR USE QR VALUES

		det_digit = None
		if self.DETECT_DIGITS:
			det_digit = qr_extr.getLastNumber()
		if det_digit:
			self.print_message("yellow", f"Using DETECTED DIGIT {det_digit} for age.")
			self.faces[i]["info"]["age"] = det_digit
		else:
			self.print_message("white", "No digit detection, using QR info for age.")

		if "mask" in self.faces[i]:
			self.print_message("yellow", "Using DETECTED value for hasMask.")
			self.faces[i]["info"]["hasMask"] = self.faces[i]["mask"]
		else:
			self.print_message("white", "No detection for hasMask, using value from QR.")

		has_mask = self.faces[i]["info"]["hasMask"]
		is_vaccinated = self.faces[i]["info"]["isVaccinated"]
		if (not has_mask):
			if (not is_vaccinated):
				self.print_message("red", "WARNING: YOU SHOULD WEAR A MASK!")
			else:
				self.print_message("green", "NO MASK BUT VACCINATED")
		else:
			self.print_message("green", "THANK YOU FOR WEARING A MASK")

		#arm_mover.extend_retract()
		appr.moveBackType("cylinder")
		qr_extr.detect_digits = False

	def approach_one_cylinder(self, color, appr, arm_mover, qr_extr):
		pos = self.cylinders[color]
		print(f"Approaching {color} doctor.")
		appr.approachnew(pos.x, pos.y, "cylinder")
		#appr.leftRight(20)
		#dataset = qr_extr.getLastDataset()
		self.print_message(color, f"HELLO, {color} doctor.")

		# GET VACCINE CLASSIFICATION OR USE VALUE FROM QR
		cls_vaccine = None
		#cls_vaccine = qr_extr.getLastClassifedVaccine # [TODO]
		if cls_vaccine:
			self.print_message("white", "Using CLASSIFIED color of vaccine.")
			self.faces[i]["info"]["vaccine"] = self.faces[i]["mask"]
		else:
			self.print_message("white", "No classification of vaccine, using value from QR.")

		appr.moveBackType("cylinder")

	def approach_one_ring(self, color, appr, arm_mover):
		pos = self.rings[color]
		print(f"Approaching {color} vaccine.")
		appr.approachnew(pos.x, pos.y, "cylinder")
		self.print_message(color, f"PICKING UP {color} vaccine.")
		arm_mover.extend_retract()

	def vaccinate_one_face(self, i, appr, arm_mover, qr_extr):
		face = self.faces[i]
		color = face["info"]["vaccine"]
		#name = face["name"]
		name = f"face {i}"
		print("Approaching " + name + ".")
		pos = face["position"]
		appr.approachnew(pos.x, pos.y, "obraz")
		self.print_message(color, f"GIVING {face['info']['vaccine']} VACCINE TO {name}")
		appr.moveForward(0.15)
		arm_mover.extend_retract()
		appr.moveBackType("cylinder")

	def approach_loop(self, i, appr, arm_mover, qr_extr):
		# VISIT ONE FACE, CYLINDER AND RING
		self.approach_one_face(i, appr, arm_mover, qr_extr) # <=============

		if self.faces[i]["info"]["isVaccinated"]:
			self.print_message("green", "THIS PERSON HAS BEEN VACCINATED ALREADY, MOVING ON...")
			return True

		next_cyl = self.faces[i]["info"]["doctor"] if self.faces[i]["info"] else None
		if not next_cyl:
			print("Doctor not known!")
			return False

		self.approach_one_cylinder(next_cyl, appr, arm_mover, qr_extr) # <=============

		next_ring = self.faces[i]["info"]["vaccine"] if self.faces[i]["info"] else None
		if not next_ring:
			print("Vaccine not known!")
			return False

		self.approach_one_ring(next_ring, appr, arm_mover) # <=============

		self.vaccinate_one_face(i, appr, arm_mover, qr_extr) # <=============

		return True


	# MAIN RUNTIME FUNCTION
	def runtime(self):
		#self.faces = self.test_faces
		#self.cylinders = self.test_cylinders
		#self.rings = self.test_rings
		self.CHECK_WALL = False#whether to check for wall between faces
		self.DETECT_DIGITS = True
		self.CLASSIFY_VACCINE = False

		# SHADOWS OFF IN GAZEBO!

		# EXPLORE PHASE CLASSES
		cylinder_f = CylinderFilter()
		face_ring_f = FaceRingWrapper()

		#self.explore_step(cylinder_f, face_ring_f)  # <=============

		cylinder_f.disable()
		face_ring_f.disable()
		self.kill_nodes(["cylinder_segmentation", "face_localizer", "face_detector_node", "ring_detector"])

		# BACKUP
		num_obj = 4
		if (not self.faces) or (len(self.faces) < num_obj):
			self.print_message("red", "FACE detections not complete, using hardcoded value for demonstration.")
			self.faces = self.test_faces
		if (not self.cylinders) or (len(self.cylinders) < num_obj):
			self.print_message("red", "CYLINDER detections not complete, using hardcoded value for demonstration.")
			self.cylinders = self.test_cylinders
		if (not self.rings) or (len(self.rings) < num_obj):
			self.print_message("red", "RING detections not complete, using hardcoded value for demonstration.")
			self.rings = self.test_rings


		# APPROACH PHASE CLASSES
		appr = Approacher()
		arm_mover = Arm_Mover()
		qr_extr = QRExtractor()
		qr_extr.detect_digits = False
		#qr_extr.visualize = True

		self.warning_step(appr)  # <=============

		failed_faces = [] #indices for faces where there was an error, to try again

		for i in range(0, len(self.faces)):

			ret = False
			try:
				ret = self.approach_loop(i, appr, arm_mover, qr_extr) # <=============
			except Exception as e:
				print(e)

			if not ret:
				self.print_message("red", "THERE WAS A PROBLEM, WILL TRY THIS FACE AGAIN LATER")
				failed_faces.append(i)
				continue

			print(f"Approach loop for face {i} was successful.")

		for i in failed_faces:
			print(f"Trying face {i} again.")
			ret = False
			try:
				ret = self.approach_loop(i, appr, arm_mover, qr_extr) # <=============
			except Exception as e:
				print(e)
			if not ret:
				self.print_message("red", "THERE WAS A PROBLEM AGAIN, NEXT FACE")
				continue

		print("All done")

	def test(self):
		print("hello world")


if __name__ == "__main__":
	a = Agent()
	a.runtime()
