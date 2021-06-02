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


DIST_THR = 0.25
NUM_THR = 4


class CylinderFilter:

    # detection = {
    #     "data": Detection.msg: {position, num_detections, red, green, blue},
    #     "num_detects"
    #     "img": image from detected points,
    #     "hist": hsv hist from img,
    #     "color": classified color (string),
    #     }

    def __init__(self):
        self.detections = []  # dicts for all detections
        self.true_detections = []  # only verified detections
        #-> currently done with get_true_detections

        if __name__ == "__main__":
            rospy.init_node("cylinder_filter", anonymous=False)
        #self.detected_cylinder_sub = rospy.Subscriber("/detected_cylinder", Marker, self.on_detect) #temp
        self.detected_info_sub = rospy.Subscriber("/detected_cylinder_info", Detection, self.on_detect)

        #markers_topic = rospy.get_param('~markers_topic', 'crumbs')
        self.marker_pub = rospy.Publisher('filtered_cylinder', MarkerArray, queue_size=5)
        self.marker_pub.publish(MarkerArray())

    def disable(self):
        self.detected_info_sub.unregister()

    # UTILITY

    def hsv_hist(self, img, bins):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        mask = img_hsv[:, :, 1] > 30  # ONLY KEEP PIXELS THAT ARE NOT COMPLETELY DESATURATED
        h,bins = np.histogram(img_hsv[:, :, 0][mask].ravel(), bins, [0, 256])
        s,bins = np.histogram(img_hsv[:, :, 1][mask].ravel(), bins, [0, 256])
        v,bins = np.histogram(img_hsv[:, :, 2][mask].ravel(), bins, [0, 256])

        return np.concatenate([h,s,v], axis=0)

    def data_to_hist(self, data):
        n = len(data.red)
        img = np.zeros([1, n, 3], dtype='uint8')
        img[0, :, 0] = np.array(list(data.red), dtype='uint8')
        img[0, :, 1] = np.array(list(data.green), dtype='uint8')
        img[0, :, 2] = np.array(list(data.blue), dtype='uint8')
        # TODO: MASK OUT DESATURATED PIXELS

        hist = self.hsv_hist(img, 10)
        return hist, img

    def show_images(self, detections):
        fig = plt.figure(figsize=(4, 16))
        n = len(detections)
        for i in range(0,n):
            fig.add_subplot(2*n, 1, 2*i+1)
            img = cv2.resize(detections[i]["img"], (128, 16))
            plt.imshow(img)
            plt.axis('off')
            # put under if
            hist_img = np.tile(detections[i]["hist"], (2,1))
            fig.add_subplot(2*n, 1, 2*i+2)
            plt.imshow(hist_img)
        plt.show()

    def print_positions(self, detections):
        print("\n\n Confirmed cylinders: ")
        for i,d in enumerate(detections):
            pos = d["data"].position
            col = d["color"] if d["color"] else ""
            print("(%.2f, %.2f, %.2f) %s" % (pos.x, pos.y, pos.z, col))
        print("---------------------------\n\n")

    def classify_color_mini(self, histogram):
            hist = np.array(histogram)  # requires 10bit/channel hsv hist
            colors = ["red", "yellow", "green", "blue"]
            sums = [np.sum(hist[0:1]), np.sum(hist[1:2]), np.sum(hist[2:4]), np.sum(hist[3:6])]# <----
            #print(sums)
            index = np.argmax(sums)
            color = colors[index]
            # no free lunch for the win
            return color

    # METHODS

    def on_detect(self, data):
        now_sec = rospy.get_time()
        pos = data.position
        rospy.loginfo("(%.2f, %.2f, %.2f) recieved at %.0f" % (pos.x, pos.y, pos.z, now_sec) )
        rospy.loginfo(data.num_detections)

        self.process_detection(data)

    def process_detection(self, data):
        pos = data.position

        for i in range(0, len(self.detections)):
            pos2 = self.detections[i]["data"].position
            dist = math.hypot(pos.x-pos2.x, pos.y-pos2.y)

            # if close enough to a previous detection
            if dist < DIST_THR:
                self.detections[i]["num_detects"] += 1
                #print(".")
                if self.detections[i]["num_detects"] == NUM_THR:
                    # temp - MEGA BODGE, CLEANUP LATER (something like classify_detect((data1, data2, ..)))
                    hist, img = self.data_to_hist(data)
                    d2 = self.detections[i]
                    new_img = np.concatenate((img, d2["img"]), axis=1)
                    new_hist = hist + d2["hist"]
                    self.detections[i]["img"] = new_img
                    self.detections[i]["hist"] = new_hist
                    self.detections[i]["color"] = self.classify_color_mini(new_hist)
                    d2["data"].position.x = (d2["data"].position.x + data.position.x) / 2
                    d2["data"].position.y = (d2["data"].position.y + data.position.y) / 2
                    # we have a verified detection
                    true_det = self.get_true_detections() # idk
                    self.print_positions(true_det)
                    self.pub_markers(true_det)
                    #self.show_images(true_det)
                return

        # if detection is new
        hist, img = self.data_to_hist(data) #idk if this goes here or on confirm
        color = self.classify_color_mini(hist)

        new_detection = {
            "data": data,
            "num_detects": 1,
            "hist": hist,
            "img": img,
            "color": color
        }
        self.detections.append(new_detection)
        #print("$")

    # return only verified detections
    def get_true_detections(self):
        return [d for d in self.detections if d["num_detects"] >= NUM_THR]
        # true_det = []
        # for d in self.detections:
        #     if d["num_dete"]

    # return n points with most repeated detections
    def get_topn_detections(self, n):
        sort_by = lambda d: d["num_detects"]
        temp = sorted(self.detections, key=sort_by, reverse=True)
        return temp[0:n]

    def pub_markers(self, detections):
        markers = MarkerArray()

        colors = {
            "red": ColorRGBA(1, 0, 0, 1),
            "green": ColorRGBA(0, 1, 0, 1),
            "blue": ColorRGBA(0, 0, 1, 1),
            "yellow": ColorRGBA(1, 1, 0, 1),
        }

        i = 0
        for d in detections:
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'map'
            marker.pose.position = d["data"].position
            marker.pose.orientation = Quaternion(0, 0, 0, 1)
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.frame_locked = False
            marker.lifetime = rospy.Time(0)
            marker.id = i
            marker.scale = Vector3(0.2, 0.2, 0.2)
            marker.color = colors[d["color"]]
            markers.markers.append(marker)
            i = i + 1

        self.marker_pub.publish(markers)

    def get_final_detections(self):
        final_det = {}
        true_det = self.get_true_detections()
        #true_det = self.get_topn_detections(4)
        for d in true_det:
            final_det[d["color"]] = d["data"].position

        return final_det

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    cf = CylinderFilter()
    rospy.spin()
