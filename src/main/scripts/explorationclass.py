#!/usr/bin/python3

import sys
import rospy
import actionlib
import math


import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.srv import GetPlan
from skimage import img_as_bool, io, color, morphology, feature
import matplotlib.pyplot as plt

class explorer():
	def __init__(self):
		self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.client.wait_for_server()

		self.get_plan = rospy.ServiceProxy('move_base/NavfnROS/make_plan', GetPlan)
		self.publishnavi=rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
		
		self.points=[]
		self.rotate=False


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

	def rotateFor(self,relative_angle_deg,angular_speed=0.75):
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
		    publishnavi.publish(msgTwist)
		    t1=rospy.Time.now().to_sec()
		    current_angle=angular_speed*(t1-t0)
		    
		rospy.loginfo("Rotation ended")
		
		msgTwist.angular.z=0
		publishnavi.publish(msgTwist)
		
		return



	def getangle(self,deg):
		#mu das stopinje in on vrne kvaterion rotacije za ta kot
		quat=tf.transformations.quaternion_from_euler(0,0,math.radians(deg))
		return quat    
		

	def moveTo(self,locx,locy,angle=1):

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
			self.client.wait_for_result(rospy.Duration(1))
		
		if self.client.get_state() == 3:
			print("Goal reached")
		else:
			print("Unable to reach goal, moving on")
			moveBack(0.3)
			return False
			
		
		print("Reached",locx,"  ",locy)

		return

	def getNear(self,cropped,i,j):
		count=0
		#print(i,j)
		temp=[cropped[i+1,j+1],cropped[i+1,j],cropped[i+1,j-1],cropped[i,j+1],cropped[i,j-1],cropped[i-1,j+1],cropped[i-1,j],cropped[i-1,j-1]]
		count=temp.count(True)
		#print(count)
		return count

	def getLocs(self,cropped):

		lokacije=[]
		(visina,sirina)=cropped.shape
		num=0
		
		for i in range(1,visina-1):
			for j in range(1,sirina-1):
				if cropped[i,j]==True:
					num=self.getNear(cropped,i,j)
					if num>=3:
						lokacije.append([i,j])
						
		
		return lokacije

	def getDist(self,x1,y1,x2,y2):

		dist=math.hypot(x2-x1,y2-y1)

		return dist
	
	def moveToAllGoals(self):
		#dejansko poslji movements
		for i in self.points:
			if self.checkGoal(0,0,i[0],i[1])!=[]:
				test=self.moveTo(i[0],i[1])
				if(test==False):
					#try again
					self.moveTo(i[0],i[1])
				if(self.rotate):
					rotateFor(345,1)
			else:
				print("something went wrong with: ",i)
		
	
	def getAllGoals(self):
		return self.points
		
	def goToGoal(self,i):
		if self.checkGoal(0,0,i[0],i[1])!=[]:
			test=self.moveTo(i[0],i[1])
			if(test==False):
				#try again
				self.moveTo(i[0],i[1])
			if(self.rotate):
				rotateFor(345,1)
				return True
			else:
				print("something went wrong with: ",i)
				return False
				
	def toggleRotate(self):
		self.rotate=not self.rotate
		return self.rotate
	
	def preslikaj(self,loc):
		#map x iz 0,110 v -1.6,4
		#map y iz 0,100 v 2.8,-2.15 // razpon 0,4.95
		#prej sm jih dav obrjena nt 
		y=loc[0]
		x=loc[1]
		#print("old x:",x,"old y",y)
		newx=-1.6+((4+1.6)/(110-0))*(x-2)
		newy=2.9-((4.95-0)/(100-0))*(y-0)
		newloc=[newx,newy]
		#print("new x:",newx,"new y",newy)
		return newloc
		
	def sortgoals(self,goals):
		startx=0
		starty=0
		goalscpy=goals.copy()
		newgoals=[]
		#print(goals)
		while(len(goalscpy)>2):
			#find closest
			minlen=999999
			for i in goalscpy:
				if(self.getDist(startx,starty,i[0],i[1])<minlen):
					minlen=self.getDist(startx,starty,i[0],i[1])
					current=i
			#remove from cpy and add to new list
			newgoals.append(current)
			goalscpy.remove(current)
			startx=current[0]
			starty=current[1]
		#zadna dva dodamo manually ce ne se zmede ko je samo left
		newgoals.append(goalscpy.pop(1))
		newgoals.append(goalscpy.pop(0))
		#print("new goals:",newgoals)
		return newgoals

	def explore(self):
		
		rospy.loginfo("opening and analysing map")
		image=io.imread('/home/bor/rins-kappa/src/navigation_stack/maps/ok_map.pgm')##PATH HARDCODED
		binary=image > 220
		skeleton=morphology.skeletonize(binary)
		cropped=skeleton[180:280,210:320]
		
		locations=[]
		locations=self.getLocs(cropped)
		rospy.loginfo("filtering potential coordinates")
		locationscpy=locations.copy()
		size=len(locations)
		#print(locations)
		for i in range(size):
			for j in range(i,size):
				prvi=locationscpy[i]
				drugi=locationscpy[j]
				if self.getDist(prvi[0],prvi[1],drugi[0],drugi[1])<10 and prvi != drugi:
					
					locations.remove(prvi)
					break

		
		#preslikava iz ttih v koordinate mape
		trueLocations=[]
		for i in locations:
			trueLocations.append(self.preslikaj(i))
		rospy.loginfo("got coordinates")
		#print(trueLocations)

		# sortiraj da bojo sli do najblizje
		trueLocations=self.sortgoals(trueLocations)
		
		self.points=trueLocations
		
		
		self.moveToAllGoals()

		print("Done")
		
if __name__ == "__main__":
	rospy.init_node("explorenode1")
	#explore naredi vse. ce hoces delat manually goal by goal lahko zakomentiras self.moveToAllGoals() v explore pol dobis s getAllGoals cilje in pol loopas cez njih in za vsakega posles goToGoal(). goToGoal rabi zap.stevilo cilja in vrne true ce pride do tja in false ce ne.
	ex=explorer()
	ex.explore()
