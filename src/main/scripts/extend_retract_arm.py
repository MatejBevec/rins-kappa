#!/usr/bin/python3

import rospy
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String

class Arm_Mover():
    def __init__(self):

        rospy.init_node('extend_retract_arm', anonymous=True)

        self.arm_movement_pub = rospy.Publisher('/turtlebot_arm/arm_controller/command', JointTrajectory, queue_size=1)

        # Pre-defined positions for the arm
        self.retract = JointTrajectory()
        self.retract.joint_names = ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_flex_joint", "arm_wrist_flex_joint"]
        self.retract.points = [JointTrajectoryPoint(positions=[0,-1.3,2.2,1],
                                                    time_from_start = rospy.Duration(1))]

        self.extend = JointTrajectory()
        self.extend.joint_names = ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_flex_joint", "arm_wrist_flex_joint"]
        self.extend.points = [JointTrajectoryPoint(positions=[0,1.10,0.25,0.15],
                                                    time_from_start = rospy.Duration(1))]

    def extend_retract(self):
        self.arm_movement_pub.publish(self.extend)
        time.sleep(3)
        self.arm_movement_pub.publish(self.retract)
        time.sleep(1)

if __name__ == "__main__":

    am = Arm_Mover()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        input("press any key")
        print("extending")
        am.extend_retract()
        print("retracting")
