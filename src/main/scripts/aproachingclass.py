#!/usr/bin/python3

import sys
import rospy
import actionlib
import math


import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.srv import GetPlan

class Approacher():
	def __init__(self):
		self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.client.wait_for_server()

		self.get_plan = rospy.ServiceProxy('move_base/NavfnROS/make_plan', GetPlan)
		self.publishnavi=rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)


	def moveBack(self,dist):
		speed=-0.2

		msgTwist=Twist()
		msgTwist.linear.x=speed
		msgTwist.linear.y=0
		msgTwist.linear.z=0
		msgTwist.angular.x=0
		msgTwist.angular.y=0
		msgTwist.angular.z=0

		t0=rospy.Time.now().to_sec()
		current_dist=0

		while(current_dist<dist):
		    self.publishnavi.publish(msgTwist)
		    t1=rospy.Time.now().to_sec()
		    current_dist=-1*speed*(t1-t0)

		rospy.loginfo("movement ended")

		msgTwist.linear.x=0
		self.publishnavi.publish(msgTwist)
		return

	def moveForward(self,dist):

		speed=0.2

		msgTwist=Twist()
		msgTwist.linear.x=speed
		msgTwist.linear.y=0
		msgTwist.linear.z=0
		msgTwist.angular.x=0
		msgTwist.angular.y=0
		msgTwist.angular.z=0

		t0=rospy.Time.now().to_sec()
		current_dist=0

		while(current_dist<dist):
		    self.publishnavi.publish(msgTwist)
		    t1=rospy.Time.now().to_sec()
		    current_dist=speed*(t1-t0)

		rospy.loginfo("movement ended")

		msgTwist.linear.x=0
		self.publishnavi.publish(msgTwist)
		return

	def rotateFor(self,relative_angle_deg,angular_speed=0.35):
	#rotateFor zavrti robota za relative_angle_deg stopinj s hitrostjo angular_speed
	#ce je kot negativen se vrti v obratno smer

		isnegative=1;
		if(relative_angle_deg>360):
			relative_angle_deg=relative_angle_deg%360
		if(relative_angle_deg<0):
			relative_angle_deg=360-(relative_angle_deg%360)
			isnegative=-1

		relative_angle=relative_angle_deg*2*math.pi/360
		rospy.loginfo("Rotating for %f rad == %f deg",relative_angle,relative_angle_deg)

		msgTwist=Twist()

		msgTwist.linear.x=0
		msgTwist.linear.y=0
		msgTwist.linear.z=0
		msgTwist.angular.x=0
		msgTwist.angular.y=0
		msgTwist.angular.z=angular_speed*isnegative


		t0=rospy.Time.now().to_sec()
		current_angle=0

		while(current_angle<relative_angle):
			self.publishnavi.publish(msgTwist)
			t1=rospy.Time.now().to_sec()
			current_angle=angular_speed*(t1-t0)

		rospy.loginfo("Rotation ended")

		msgTwist.angular.z=0
		self.publishnavi.publish(msgTwist)

		return


	def checkGoal(self,currentx,currenty,goalx,goaly):
		# checkGoal preveri ce je goal reachable iz current pozicije
		rospy.loginfo("Checking goal x: %f, y: %f.",goalx,goaly)


		current=PoseStamped()
		current.header.seq=0
		current.header.frame_id= "map"
		current.header.stamp=rospy.Time(0)
		current.pose.position.x=currentx
		current.pose.position.y=currenty

		current.pose.orientation.z=0.0
		current.pose.orientation.w=0.0


		goal=PoseStamped()
		goal.header.seq=0
		goal.header.frame_id= "map"
		goal.header.stamp=rospy.Time(0)
		goal.pose.position.x=goalx
		goal.pose.position.y=goaly

		goal.pose.orientation.z=0.0
		goal.pose.orientation.w=0.0


		req=GetPlan()
		req.start=current
		req.goal=goal
		req.tolerance=0.008
		resp=self.get_plan(req.start, req.goal,req.tolerance)
		planposes=resp.plan.poses


		return planposes


	def getangle(self,deg):
		#mu das stopinje in on vrne kvaterion rotacije za ta kot
		quat=tf.transformations.quaternion_from_euler(0,0,math.radians(deg))
		return quat



	def moveTo(self,locx,locy,angle):

		goal = MoveBaseGoal()

		# set next point as goal
		targetquaterion=self.getangle(angle)
		goal.target_pose.pose.position.x = locx
		goal.target_pose.pose.position.y = locy
		goal.target_pose.pose.orientation.w = targetquaterion[2]
		goal.target_pose.pose.orientation.z = targetquaterion[3]

		goal.target_pose.header.frame_id = "map"

		print("Sending goal ")

		self.client.send_goal(goal)
		while self.client.get_state() < 3:
			print("moving ...")
			#print(".")
			self.client.wait_for_result(rospy.Duration(1))

		if self.client.get_state() == 3:
			print("Goal reached")
		else:
			print("Unable to reach goal, moving on")

		print(f"Reached {locx:.2f}, {locy:.2f}")

		return

	def moveBetween(self,locx,locy,rotation=0):

		check=[]
		check=self.checkGoal(0.0,0.0,locx,locy)
		dist=0.6

		if(check!=[]):
			#ce lahko pride na prvo lokacijo rotation idk kam je treba bit obrjen
			self.moveTo(locx,locy,rotation)
			return
		else:
			ring1x=[0.0,0.0,dist,-dist]
			ring1y=[dist,-dist,0.0,0.0]
			rotations=[270,90,0,180]
			for i in range(4):
				check=self.checkGoal(0.0,0.0,locx+ring1x[i],locy+ring1y[i])
				rotation
				if(check!=[]):
					self.moveTo(locx,locy,rotations[i])
					print("2 faces are too close")
					rospy.sleep(2)
					return
		return


	def approach(self,locx,locy,tip):

		#dist nastavi kako dalec od objekta naj gleda
		distaway=0.49

		modifier=1
		if(tip == 'ring'):
			modifier=2
		if(tip == 'obraz'):
			modifier=0
			distaway=0.49 #nism cist sure kk dalec stran od obraza mora bit da lepo zazna can be changed if needed

		ring1x=[0.0,distaway,distaway,distaway,0.0,-distaway,-distaway,-distaway]
		ring1y=[distaway,distaway,0.0,-distaway,-distaway,-distaway,0.0,distaway]
		rotations=[270,305,10,60,100,145,185,235]
		check=[]
		#pogleda ce so reachable
		for i in range(8):
			check=self.checkGoal(0.0,0.0,locx+ring1x[i],locy+ring1y[i])
			if(check!=[]):
			    rospy.loginfo(f"goal {locx+ring1x[i]}, {locy+ring1y[i]} reachable")
			    self.moveTo(locx+ring1x[i],locy+ring1y[i],rotations[i])
			    if(i%2==1):
			        modifier=modifier*1.4
			    self.moveForward(0.1*modifier)
			    rospy.sleep(1) #samo za test kasnej lahko damo vn

			    self.moveBack(0.1*modifier)
			    break
			else:
			    rospy.loginfo(f"goal {locx+ring1x[i]}, {locy+ring1y[i]} unreachable")


		#self.moveTo(0,0,190)	#nazaj na zacetekzakomentiraj to ko bo konc testiranja


		rospy.loginfo("Approaching done")



	def getavgAngle(self,angles):
		x=y=0
		for i in angles:
			x += math.cos(i)
			y += math.sin(i)

		return math.atan2(y, x)

	def leftRight(self,deg):
		self.rotateFor(deg)
		self.rotateFor(deg*-2)
		self.rotateFor(deg)
		return

	def checkwall(self,x1,y1,x2,y2,numberofpoints):
		m=(y2-y1)/(x2-x1)
		b=y1-m*x1
		print("enacba je y=",m,"*x+",b)
		distx=x2-x1
		disty=y2-y1

		stepx=distx/numberofpoints
		stepy=disty/numberofpoints
		check=[]
		for i in range(1,numberofpoints):
			check=self.checkGoal(0.0,0.0,x1+i*stepx,y1+i*stepy)
			if(check==[]):
				return False
		return True

	def approachnew(self,locx,locy,tip):

		#dist nastavi kako dalec od objekta naj gleda
		distaway=0.49
		facerotate=30
		modifier=1
		if(tip == 'ring'):
			modifier=2
		if(tip == 'obraz'):
			modifier=0
			distaway=0.49 #nism cist sure kk dalec stran od obraza mora bit da lepo zazna can be changed if needed

		anglereachable=[]
		check=[]
		#pogleda ce so reachable
		x=0
		y=0
		numofangles=12#stevilo kolkkrat naj pogleda a.k.a. pogleda na vsakih 360/numofangles stopinj
		for i in range(numofangles):
			x=locx+distaway*math.cos(math.radians((i*(360/numofangles))))
			y=locy+distaway*math.sin(math.radians((i*(360/numofangles))))
			check=self.checkGoal(0.0,0.0,x,y)
			if(check!=[]):
				anglereachable.append(i*(360/numofangles))

		rospy.loginfo(anglereachable)
		#print("new angle",(self.getavgAngle(anglereachable)+360)%360)


		avgangle=0
		#ce ni pogledamo povprecni kot kamor ne more prit
		if(anglereachable!=[]):
			avgangle=sum(anglereachable)/len(anglereachable)
		rospy.loginfo(avgangle)
		#just in case
		avgangle=(avgangle)%360
		#dejanski kot kam rabi it
		rospy.loginfo(avgangle)


		goalx=locx+distaway*math.cos(math.radians(avgangle))
		goaly=locy+distaway*math.sin(math.radians(avgangle))
		rospy.loginfo(f"goal: {goalx}, {goaly}")
		check=self.checkGoal(0.0,0.0,goalx,goaly)
		if(check==[]):
			avgangle=self.getavgAngle(anglereachable)
			avgangle=(avgangle)%360
			goalx=locx+distaway*math.cos(math.radians(avgangle))
			goaly=locy+distaway*math.sin(math.radians(avgangle))
			check=self.checkGoal(0.0,0.0,goalx,goaly)
			if (check==[]):
				#se zadnji failsafe
				newangle=min(myList, key=lambda x:abs(x-avgangle))
				goalx=locx+distaway*math.cos(math.radians(newangle))
				goaly=locy+distaway*math.sin(math.radians(newangle))
				avgangle=newangle
				#print("nov kot",newangle)
		self.moveTo(goalx,goaly,abs(avgangle-360)%360)
		rospy.sleep(1)

		if(tip=='cylinder'):
			self.moveForward(0.15)
			self.moveBack(0.15)



		#ko pride do cilja pogleda malo naokoli da najde qr ce se ne bo klicalo iz kod drugot samo odzakomentiraj ta if
		#if(tip=='obraz'):
			#self.leftRight(facerotate)


		#self.moveTo(0,0,190)	#nazaj na zacetek TODO: zakomentiraj to ko bo konc testiranja


		rospy.loginfo("Approaching done")

	def moveBackType(self, tip):
		modifier=1
		if(tip == 'ring'):
			modifier=2
		if(tip == 'obraz'):
			modifier=0

		self.moveBack(0.1*modifier)

if __name__ == "__main__":
	rospy.init_node("navnode2")
	apr=Approacher()
	#apr.approach(-0.5,0.0,'cylinder')#neutral
	#apr.approach(2.45,2.65,'cylinder')#cylinder
	#apr.approach(-1.33,0.25,'ring') #ring
	#apr.moveBetween(1.15,1.3)
	#apr.moveBetween(0.1,0.1)
	#apr.approachnew(0.0,0.0,'ring')
	#apr.approachnew(-1.8,-0.2,'cylinder')
	#apr.approachnew(2.9,0.4,'cylinder')
	#apr.approachnew(2.5,2.6,'cylinder')
	#apr.approachnew(-0.7,-1.85,'cylinder')
	apr.approachnew(1.3,2.6,'obraz')
	apr.approachnew(4,-1.0,'obraz')
	#apr.approachnew(-1.1,0.6,'ring')
	#apr.approachnew(-0.2,1.35,'ring')
	#apr.approachnew(2.9,-1.55,'ring')
	#apr.approachnew(2.2,1.4,'ring')
	apr.approachnew(0.5,0.34,'obraz')
	apr.approachnew(0.8,1.2,'obraz')
	#print(apr.checkwall(0.5,0.34,0.8,1.2,5))
	#print(apr.checkwall(-0.7,1.25,0.2,-1.34,5))
