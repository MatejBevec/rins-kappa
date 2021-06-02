#!/usr/bin/python3

import sys
import rospy
import actionlib
import math

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class FaceRingWrapper:

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
                "mask": pos.y < 0.3
            }
            self.faces.append(face)
        #self.faces = latest_faces

    def color_to_string(self, color):
        if color == ColorRGBA(1, 0, 0, 1):
            return "red"
        if color == ColorRGBA(0, 1, 0, 1):
            return "green"
        if color == ColorRGBA(0, 0, 1, 1):
            return "blue"
        if color == ColorRGBA(1, 1, 0, 1):
            return "yellow"
        if color == ColorRGBA(1, 1, 1, 1):
            return "white"
        if color == ColorRGBA(0, 0, 0, 1):
            return "black"

        return "red" #if all else fails

    def on_confirmed_ring(self, data):
        self.rings = []
        for marker in data.markers:
            pos = marker.pose.position
            color = marker.color
            print(f"Confirmed RING detection at ({pos.x},{pos.y})")
            ring = {
                "position": pos,
                "color": self.color_to_string(color)
            }
            self.rings.append(ring)
        pass

    def get_final_face_detections(self):
        return self.faces

    def get_final_ring_detections(self):
        return self.rings


if __name__ == "__main__":
    rospy.init_node("face_filter", anonymous=False)
    ff = FaceFilter()
    rospy.spin()
