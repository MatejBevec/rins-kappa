#!/usr/bin/python3

import rospy
import time
from std_msgs.msg import String

def extend_rectract_arm(arm_command_pub):
    msg = String()
    msg.data = 'extend'
    rospy.arm_command_pub.publish(msg)
    time.sleep(3)
    msg.data = 'retract'
    rospy.arm_command_pub.publish(msg)
    time.sleep(1)

if __name__ == "__main__":
    rospy.init_node('extend_arm', anonymous=True)

    arm_command_pub = rospy.Publisher("/arm_command", String)
    while True:
        input()
        extend_rectract_arm(arm_command_pub)
