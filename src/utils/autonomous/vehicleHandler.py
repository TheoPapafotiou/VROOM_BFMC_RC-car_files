# Reaction to static elements code

import cv2
import numpy as np
import os
import imutils
from imutils.object_detection import non_max_suppression
from time import sleep 
import math

class VehicleHandler:

	def __init__(self):	

		self.ccascade = cv2.CascadeClassifier('src/utils/autonomous/cars.xml')
		self.bbox = (0,0,0,0)
		self.vehicleDetected = False

	def detect_vehicle(self, image):
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		cars = self.ccascade.detectMultiScale(gray, 1.5, 14)

		for (x,y,w,h) in cars:
			detected_area =  (x+w)*(y+h)
			# if x > 300 and x < 500 and y>=44 and detected_area > 70000:
			self.vehicleDetected = True
			self.bbox = (x,y,w,h)

		cars = np.array([[x, y, x + w, y + h] for (x, y, w, h) in cars])
		pick = non_max_suppression(cars, probs=None, overlapThresh=0.6)

		for(xA, yA, xB, yB) in pick:
			#if (xA+xB) * (yA + yB) > 70000:
			cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)

		return self.vehicleDetected

	# def distance_to_vehicle(self, frame):
	# 	img_dims = frame[:,:,0].shape

	# 	roi_center = (self.bbox[0] + self.bbox[2]/2, self.bbox[1] + self.bbox[3]/2) 
	# 	vehicle_nose = (img_dims[1]/2, img_dims[0])

	# 	cv2.line(frame, vehicle_nose, roi_center, (255,0,0), 3)

	# 	vehicle_nose_list = [img_dims[1] / 2, img_dims[0]]
	# 	roi_center_list = [self.bbox[0] + self.bbox[2]/2, self.bbox[1] + self.bbox[3]/2]

	# 	self.distance = vehicle_nose_list[1] - roi_center_list[1]
	# 	print("Distance to vehicle = ", self.distance)
