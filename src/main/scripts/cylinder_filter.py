#!/usr/bin/python3

import sys
import rospy
import math
import numpy as np
import cv2

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import String, Bool, ColorRGBA

from exercise6.msg import Detection

import matplotlib.pyplot as plt


# self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
# self.markers_pub = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)


# def detected_new_face(pos):
#     for face in faces:
#         if dist(face.pos, pos) < thr:
#             found_cluster = True
#             face.count += 1
#             if face.count > 10:
#                 mark_as_true_and_approach()
#     if not found_cluster:
#         faces.append(detected face)

DIST_THR = 0.25
NUM_THR = 4



class CylinderFilter():

    def __init__(self):
        self.positions = [] #locations of filtered cylinder detections
        self.num_detects = []
        self.histograms = [] #histograms of detected point cloud segments
        self.images = []
        self.data = [] #Detection message for each new detection
        self.colors = []

        rospy.init_node("cylinder_filter", anonymous=False)
        #self.detected_cylinder_sub = rospy.Subscriber("/detected_cylinder", Marker, self.on_detect) #temp
        self.detected_info_sub = rospy.Subscriber("/detected_cylinder_info", Detection, self.on_detect_info)

        #markers_topic = rospy.get_param('~markers_topic', 'crumbs')
        self.marker_pub = rospy.Publisher('filtered_cylinder', MarkerArray, queue_size=5)
        self.marker_pub.publish(MarkerArray())

    # wrapper for process_detection
    def on_detect(self, data):
        pos = data.pose.position
        now_sec = rospy.get_time()
        #print("(%.2f, %.2f, %.2f) recieved at %.0f" % (pos.x, pos.y, pos.z, now_sec))
        self.process_detection(data)

    def on_detect_info(self, data):
        pos = data.position
        now_sec = rospy.get_time()
        print("(%.2f, %.2f, %.2f) recieved at %.0f" % (pos.x, pos.y, pos.z, now_sec) )
        print(data.num_detections)

        self.process_detection(data)


    # check if this is a new detection or an already detected object
    def process_detection(self, data):
        pos = data.position
        for i in range(0, len(self.positions)):
            pos2 = self.positions[i]
            dist = math.hypot(pos.x-pos2.x, pos.y-pos2.y)

            if dist < DIST_THR:
                self.num_detects[i] += 1
                print(".")
                if self.num_detects[i] == NUM_THR:
                    # we have a verified detection
                    self.print_positions(self.get_filtered_positions())
                    self.pub_markers()

                    fig = plt.figure(figsize=(4,16))
                    n = len(self.images)
                    for i in range(0,n):
                        fig.add_subplot(n, 1, i+1)
                        img = cv2.resize(self.images[i], (128, 16))
                        plt.imshow(img)
                        plt.axis('off')
                    plt.show()

                return;

        self.positions.append(pos)
        self.num_detects.append(1)
        hist, img = data_to_hist(data)
        self.histograms.append(hist)
        self.images.append(img)
        print("$")

    # return only verified detections
    def get_filtered_positions(self):
        true_pos = []
        for i in range(0, len(self.positions)):
            if self.num_detects[i] >= NUM_THR:
                true_pos.append(self.positions[i])
        return true_pos

    def print_positions(self, positions):
        print("\n\n Filtered detections: ")
        for i,pos in enumerate(positions):
            print("(%.2f, %.2f, %.2f)" % (pos.x, pos.y, pos.z))
        print("---------------------------\n\n")
        print(self.num_detects)

    def pub_markers(self):
        markers = MarkerArray()
        true_pos = self.get_filtered_positions()

        i = 0
        for pos in true_pos:
            #print point
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'map'
            marker.pose.position = pos
            #marker.pose.position = Point(point.transform.translation.x, point.transform.translation.y, point.transform.translation.z)
            marker.pose.orientation = Quaternion(0, 0, 0, 1)
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.frame_locked = False
            marker.lifetime = rospy.Time(0)
            marker.id = i
            marker.scale = Vector3(0.2, 0.2, 0.2)
            marker.color = ColorRGBA(1, 1, 0, 1)
            markers.markers.append(marker)
            i = i + 1

        self.marker_pub.publish(markers)



if __name__ == "__main__":
    cf = CylinderFilter()
    rospy.spin()
