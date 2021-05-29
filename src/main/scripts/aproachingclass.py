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
		global client
		client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		client.wait_for_server()
	
		global get_plan
		get_plan=rospy.ServiceProxy('move_base/NavfnROS/make_plan', GetPlan)
		global publishnavi
		publishnavi=rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
        

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
		    publishnavi.publish(msgTwist)
		    t1=rospy.Time.now().to_sec()
		    current_dist=-1*speed*(t1-t0)
		    
		rospy.loginfo("movement ended")
		
		msgTwist.linear.x=0
		publishnavi.publish(msgTwist)
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
		    publishnavi.publish(msgTwist)
		    t1=rospy.Time.now().to_sec()
		    current_dist=speed*(t1-t0)
		    
		rospy.loginfo("movement ended")
		
		msgTwist.linear.x=0
		publishnavi.publish(msgTwist)
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
		resp=get_plan(req.start, req.goal,req.tolerance)
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
		
		client.send_goal(goal)
		while client.get_state() < 3:
			print("moving ...")
			client.wait_for_result(rospy.Duration(1))
		
		if client.get_state() == 3:
			print("Goal reached")
		else:
			print("Unable to reach goal, moving on")
		
		print("Reached",locx,"  ",locy)

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
			    print("goal",locx+ring1x[i]," ",locy+ring1y[i],"reachable")
			    self.moveTo(locx+ring1x[i],locy+ring1y[i],rotations[i])
			    if(i%2==1):
			        modifier=modifier*1.4	
			    self.moveForward(0.1*modifier)
			    rospy.sleep(2)
			    if(tip=='cylinder'):
			        #TODO integracija premikanje roke
			        print("roka")
			    self.moveBack(0.1*modifier)
			    break	
			else:
			    print("goal",locx+ring1x[i]," ",locy+ring1y[i],"unreachable")
			
		
		self.moveTo(0,0,190)	#nazaj na zacetek TODO: zakomentiraj to ko bo konc testiranja
		

		print("Done")
	
if __name__ == "__main__":
	rospy.init_node("navnode2")
	apr=Approacher()
	apr.approach(-0.5,0.0,'cylinder')#neutral
	apr.approach(2.45,2.65,'cylinder')#cylinder
	apr.approach(-1.33,0.25,'ring') #ring
	apr.approach(1.5,2.8,'obraz') #obraz
