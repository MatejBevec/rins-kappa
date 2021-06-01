#!/usr/bin/python3

import sys
import rospy
import actionlib
import math

from visualization_msgs.msg import Marker, MarkerArray


class FaceRingWrapper():

    def __init__(self):
        self.faces_sub = rospy.Subscriber('confirmed_faces', MarkerArray, self.on_confirmed_face)
        self.rings_sub = rospy.Subscriber('confirmed_rings', MarkerArray, self.on_confirmed_ring)
        self.faces = []
        self.rings = []

    def disable(self):
        self.faces_sub.unregister()
        self.rings_sub.unregister()

    def on_confirmed_face(self, data):
        self.faces = []
        for marker in data.markers:
            pos = marker.pose.position
            print(f"Confirmed FACE detection at ({pos.x},{pos.y})")
            face = {
                "position": pos,
                "mask": False #TODO: CHECK IF MASK WAS DETECTED (pos.z ?)
            }
            self.faces.append(face)
        #self.faces = latest_faces

    def on_confirmed_ring(self, data):
        #TODO: update self.rings
        pass

    def get_final_face_detections(self):
        return self.faces

    def get_final_ring_detections(self):
        return self.rings

if __name__ == "__main__":
    rospy.init_node("face_filter", anonymous=False)
    ff = FaceFilter()
    rospy.spin()
