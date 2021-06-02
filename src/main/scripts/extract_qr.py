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
import pytesseract


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
		self.lastNumber=0 # info from last number 
		
		self.ageQR=0
		self.ageCyl=0
		self.numberAge=0

	def disable(self):
		self.image_sub.unregister()

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
			print(dataset)
			self.lastDataset = dataset
			self.ageCyl=rospy.get_rostime().secs
		else:
			#print("obraz")
			#print(data)
			info=data.split(",")
			datadict={
				"hasMask": True if info[0]=='1' else False,
				"age": int(info[1].strip()),
				"isVaccinated": True if info[2].strip()=='1' else False,
				"doctor": self.parseColor( info[3].strip() ),
				"exercise": int(info[4].strip()),
				"vaccine": self.parseColor( info[5].strip() )
			}

			print(datadict)
			self.lastDetected=datadict
			self.ageQR=rospy.get_rostime().secs
		return

	def getLastDetected(self):
		if self.visualize:
			print(self.lastDetected)
		if(abs(rospy.get_rostime().secs-self.ageQR)>10):
			#ce je starejse od 10s
			print("Qr code is too old. Try to detect again")
			return None
		return self.lastDetected

	def getLastDataset(self):
		if self.visualize:
			print(self.lastDataset)
		if(abs(rospy.get_rostime().secs-self.ageCyl)>10):
			#ce je starejse od 10s
			print("dataset is too old. Try to detect again")
			return None
		return self.lastDataset

	def getLastNumber(self):
		if self.visualize:
			print(self.lastNumber)
		
		if(abs(rospy.get_rostime().secs-self.numberAge)>10):
			#ce je starejse od 10s
			print("number is too old. Try to detect again")
			return None
		
		return self.lastNumber

	def disable(self):
		self.image_sub.unregister()

	def findDigit(self,cv_image):
		
		corners, ids, rejected_corners = cv2.aruco.detectMarkers(cv_image,dictm,parameters=params)
		    
		# Increase proportionally if you want a larger image
		image_size=(351,248,3)
		marker_side=50

		img_out = np.zeros(image_size, np.uint8)
		out_pts = np.array([[marker_side/2,img_out.shape[0]-marker_side/2],
				[img_out.shape[1]-marker_side/2,img_out.shape[0]-marker_side/2],
				[marker_side/2,marker_side/2],
				[img_out.shape[1]-marker_side/2,marker_side/2]])

		src_points = np.zeros((4,2))
		cens_mars = np.zeros((4,2))

		if not ids is None:
			if len(ids)==4:
				print('4 Markers detected')
			
				for idx in ids:
					# Calculate the center point of all markers
					cors = np.squeeze(corners[idx[0]-1])
					cen_mar = np.mean(cors,axis=0)
					cens_mars[idx[0]-1]=cen_mar
					cen_point = np.mean(cens_mars,axis=0)
				
				for coords in cens_mars:
				#  Map the correct source points
					if coords[0]<cen_point[0] and coords[1]<cen_point[1]:
						src_points[2]=coords
					elif coords[0]<cen_point[0] and coords[1]>cen_point[1]:
						src_points[0]=coords
					elif coords[0]>cen_point[0] and coords[1]<cen_point[1]:
						src_points[3]=coords
					else:
						src_points[1]=coords

				h, status = cv2.findHomography(src_points, out_pts)
				img_out = cv2.warpPerspective(cv_image, h, (img_out.shape[1],img_out.shape[0]))
				
				################################################
				#### Extraction of digits starts here
				################################################
				
				# Cut out everything but the numbers
				img_out = img_out[125:221,50:195,:]
				
				# Convert the image to grayscale
				img_out = cv2.cvtColor(img_out, cv2.COLOR_BGR2GRAY)
				
				# Option 1 - use ordinairy threshold the image to get a black and white image
				#ret,img_out = cv2.threshold(img_out,100,255,0)

				# Option 1 - use adaptive thresholding
				img_out = cv2.adaptiveThreshold(img_out,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,5)
				
				# Use Otsu's thresholding
				#ret,img_out = cv2.threshold(img_out,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
				
				# Pass some options to tesseract
				config = '--psm 13 outputbase nobatch digits'
					
				# Visualize the image we are passing to Tesseract
				cv2.imshow('Warped image',img_out)
				cv2.waitKey(1)
			
				# Extract text from image
				text = pytesseract.image_to_string(img_out, config = config)

				# Check and extract data from text
				#print('Extracted>>',text)
				text=text.strip()
				if(text.isnumeric()):
					self.lastNumber=int(text)
					self.numberAge=rospy.get_rostime().secs
					print(self.lastNumber)
					print("time of detection:",self.numberAge)
				else:
					print("not numeric",text)
					print(text)
					print(type(text))
					print(len(text))
				# Remove any whitespaces from the left and right
				text = text.strip()

				# If the extracted text is of the right length
				if len(text)==2:
					x=int(text[0])
					y=int(text[1])
					print('The extracted datapoints are x=%d, y=%d' % (x,y))
				else:
					print('The extracted text has is of length %d. Aborting processing' % len(text))
				
			else:
				print('The number of markers is not ok:',len(ids))
		else:
			print('No markers found')
		
		
		return
		

	def image_callback(self,data):
		# print('Iam here!')

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		self.findDigit(cv_image)
		
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
	#qre.visualize()

	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		rate.sleep()
		temp=qre.getLastDataset()
		print("temp",temp)


	cv2.destroyAllWindows()


if __name__ == '__main__':
	main(sys.argv)
