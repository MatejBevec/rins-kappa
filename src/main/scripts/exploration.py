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

def checkGoal(currentx,currenty,goalx,goaly):
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

def rotateFor(relative_angle_deg,angular_speed=0.25):
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



def getangle(deg):
    #mu das stopinje in on vrne kvaterion rotacije za ta kot
    quat=tf.transformations.quaternion_from_euler(0,0,math.radians(deg))
    return quat    
    

def moveTo(locx,locy,angle=1):

	goal = MoveBaseGoal()
		
	# set next point as goal
	targetquaterion=getangle(angle)
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

def getNear(cropped,i,j):
	count=0
	#print(i,j)
	temp=[cropped[i+1,j+1],cropped[i+1,j],cropped[i+1,j-1],cropped[i,j+1],cropped[i,j-1],cropped[i-1,j+1],cropped[i-1,j],cropped[i-1,j-1]]
	count=temp.count(True)
	#print(count)
	return count

def getLocs(cropped):

	lokacije=[]
	(visina,sirina)=cropped.shape
	num=0
	
	for i in range(1,visina-1):
		for j in range(1,sirina-1):
			if cropped[i,j]==True:
				num=getNear(cropped,i,j)
				if num>=3:
					lokacije.append([i,j])
					
	
	return lokacije

def getDist(x1,y1,x2,y2):

    dist=math.hypot(x2-x1,y2-y1)

    return dist
    
def preslikaj(loc):
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

def explore():

	rospy.init_node("explorenode1")
	
	global client
	client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	client.wait_for_server()
	
	global get_plan
	get_plan=rospy.ServiceProxy('move_base/NavfnROS/make_plan', GetPlan)
	global publishnavi
	publishnavi=rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
	
	rospy.loginfo("opening and analysing map")
	image=io.imread('/home/bor/rins-kappa/src/navigation_stack/maps/ok_map.pgm')
	binary=image > 220
	skeleton=morphology.skeletonize(binary)
	cropped=skeleton[180:280,210:320]
	
	locations=[]
	locations=getLocs(cropped)
	rospy.loginfo("filtering potential coordinates")
	locationscpy=locations.copy()
	size=len(locations)
	#print(locations)
	for i in range(size):
		for j in range(i,size):
			prvi=locationscpy[i]
			drugi=locationscpy[j]
			if getDist(prvi[0],prvi[1],drugi[0],drugi[1])<10 and prvi != drugi:
				
				locations.remove(prvi)
				break
				
	#print(locations)
	
	#preslikava iz ttih v koordinate mape
	trueLocations=[]
	for i in locations:
		trueLocations.append(preslikaj(i))
	rospy.loginfo("got coordinates")
	#print(trueLocations)

	#TODO sortiraj da bojo sli do najblizje
	
	#dejansko poslji movements
	for i in trueLocations:
		if checkGoal(0,0,i[0],i[1])!=[]:
			moveTo(i[0],i[1])
			rotateFor(345,0.65)
		else:
			print("something went wrong with: ",i)
	
	#display mape/skeletona mape, zanimivo za videt ampak ni nujno
	#f,(ax0,ax2,ax3)=plt.subplots(1,3)
	#ax0.imshow(image,cmap='gray')
	#ax2.imshow(skeleton,cmap='gray')
	#ax3.imshow(cropped,cmap='gray')
	#plt.show()


	print("Done")
	
if __name__ == "__main__":
	explore()
