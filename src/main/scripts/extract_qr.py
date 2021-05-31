#!/usr/bin/python3

import roslib
# roslib.load_manifest('exercise4')
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import ColorRGBA

import urllib.request
import pyzbar.pyzbar as pyzbar

dictm = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# The object that we will pass to the markerDetect function
params =  cv2.aruco.DetectorParameters_create()

print(params.adaptiveThreshConstant)
print(params.adaptiveThreshWinSizeMax)
print(params.adaptiveThreshWinSizeMin)
print(params.minCornerDistanceRate)
print(params.adaptiveThreshWinSizeStep)

# To see description of the parameters
# https://docs.opencv.org/3.3.1/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html

# You can set these parameters to get better marker detections
params.adaptiveThreshConstant = 25
adaptiveThreshWinSizeStep = 2


class QRExtractor:
	def __init__(self):

		# An object we use for converting images between ROS format and OpenCV format
		self.bridge = CvBridge()
		self.visualize = False

		# Subscribe to the image and/or depth topic
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

		self.lastDetected={}  # info from last face qr code
		self.lastDataset=[]  # info from last cylinder qr code

	def visualize(self):
		#start visualizing detections (for debugging)
		self.visualize = True

	def parseColor(self, string):
		str = string.strip().lower()
		if "red" in str:
			return "red"
		if "green" in str:
			return "green"
		if "blue" in str:
			return "blue"
		if "yellow" in str:
			return "yellow"
		if "black" in str:
			return "black"
		if "white" in str:
			return "white"
		return str

	def parseData(self,dataobj):
		#print(dataobj)
		data=dataobj.data.decode('UTF-8')
		#print(data)
		if(data.endswith('.txt')):
			#print("povezava")
			dataset=[]
			file=urllib.request.urlopen(data)
			for line in file:
				info=line.decode('UTF-8')
				#print(info)
				spplitinfo=info.split(',')
				newdata={
					"age": float(spplitinfo[0].strip()),
					"exercise": float(spplitinfo[1].strip()),
					"vaccine": self.parseColor( spplitinfo[2].strip() )
				}
				dataset.append(newdata)
			#print(dataset)
			self.lastDataset = dataset
		else:
			#print("obraz")
			#print(data)
			info=data.split(",")
			print(info)
			datadict={
				"hasMask": True if info[0]=='1' else False,
				"age": int(info[1].strip()),
				"isVaccinated": True if info[2].strip()=='1' else False,
				"doctor": self.parseColor( info[3].strip() ),
				"exercise": int(info[4].strip()),
				"vaccine": self.parseColor( info[5].strip() )
			}

			#print(datadict)
			self.lastDetected=datadict
		return

	def getLastDetected(self):
		if self.visualize:
			print(self.lastDetected)
		return self.lastDetected


	def image_callback(self,data):
		# print('Iam here!')

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

			# Find a QR code in the image
		decodedObjects = pyzbar.decode(cv_image)

		#print(decodedObjects)

		if len(decodedObjects) == 1:
			dObject = decodedObjects[0]
			#print("Found 1 QR code in the image!")
			#print("Data: ", dObject.data,'\n')
			self.parseData(dObject)

			if self.visualize:
				# Visualize the detected QR code in the image
				points  = dObject.polygon
				if len(points) > 4 :
					hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
					hull = list(map(tuple, np.squeeze(hull)))
				else :
					hull = points;

				## Number of points in the convex hull
				n = len(hull)

				## Draw the convext hull
				for j in range(0,n):
					cv2.line(cv_image, hull[j], hull[ (j+1) % n], (0,255,0), 2)

				cv2.imshow('Warped image',cv_image)
				cv2.waitKey(1)

		elif len(decodedObjects)==0:
			#print("No QR code in the image")
			pass
		else:
			#print("Found more than 1 QR code")
			pass


def main(args):

	rospy.init_node('image_converter', anonymous=True)
	qre = QRExtractor()
	qre.visualize()

	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		rate.sleep()
		temp=qre.getLastDetected()
		#print("temp",temp)


	cv2.destroyAllWindows()


if __name__ == '__main__':
	main(sys.argv)
