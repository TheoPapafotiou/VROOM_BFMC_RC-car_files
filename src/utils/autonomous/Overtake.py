import cv2
import numpy as np
import os
import imutils
from imutils.object_detection import non_max_suppression
from time import sleep 
import math
from src.utils.autonomous.LaneKeepingReloaded  import LaneKeepingReloaded

class OvertakeProcedure:

	def __init__(self):

		self.laneKeepingFlag = False
		self.angle = 0
		self.speed = 0.08
		self.distance = 0
		self.part = [False, False, False, False, False]
		self.overtake_done = False
		self.minSpeed = 0.08
		self.threshold = 30
		self.step_sub = 0.25
		self.counter = 0
		
	def distance_to_vehicle(self, frame, bbox):
		img_dims = frame[:,:,0].shape

		roi_center = (bbox[0] + bbox[2]/2, bbox[1] + bbox[3]/2) 
		vehicle_nose = (img_dims[1]/2, img_dims[0])

		cv2.line(frame, vehicle_nose, roi_center, (255,0,0), 3)

		vehicle_nose_list = [img_dims[1] / 2, img_dims[0]]
		roi_center_list = [bbox[0] + bbox[2]/2, bbox[1] + bbox[3]/2]

		self.distance = vehicle_nose_list[1] - roi_center_list[1]
		print("Distance to vehicle = ", self.distance)

	def check_dotted_line(self, graph, source, target):
		dotted = graph[source][target]["dotted"]
		return dotted

	def react_to_vehicle(self, dotted):
            
		overtake_allowed = False
		if self.distance <= self.minDistance:
			if dotted is True:
				overtake_allowed = True
				return self.speed, overtake_allowed
			else:
				return self.speed, overtake_allowed
		else:
			self.speed = self.minSpeed
			return self.speed, overtake_allowed

	def make_detour(self, yaw_init, yaw, overtake_flag):
		
		print(20*"--")
		if self.counter == 70:
			self.overtake_done = True

		if yaw == yaw_init and self.part[0] is False: # Turn left
			for i in range(0, 4):
				self.part[i] = False
			self.part[0] = True

		elif abs(yaw - yaw_init) >= self.threshold and self.part[0] is True: # Turn right to correct
			for i in range(0, 4):
				self.part[i] = False
			self.part[1] = True

		elif abs(yaw - yaw_init) in range (0,4) and self.part[1] is True: # Go straight (Lane keeping)
			for i in range(0, 4):
				self.part[i] = False
			self.part[2] = True

		elif abs(yaw - yaw_init) in range(0, 2) and self.part[2] is True and self.overtake_done is True: # Turn right 
			for i in range(0, 4):
				self.part[i] = False
			self.part[3] = True
			self.overtake_done = False
			self.counter = 0

		elif abs(yaw - yaw_init) >=  self.threshold and self.part[3] is True: #Turn left 
			for i in range(0, 4):
				self.part[i] = False
			self.part[4] = True

		elif abs(yaw - yaw_init) in range (0,1) and self.part[4] is True: # Overtake is done. Lane keeping
			for i in range(0, 4):
				self.part[i] = False
			#self.part[0] = True
			overtake_flag = False   

		### Calculate the speed and angle
		if self.part[0] is True:
			print("===================Part 0==================")
			self.angle -= self.step_sub
			self.speed = self.minSpeed

		elif self.part[1] is True:
			print("Part 1")
			self.angle += self.step_sub
			self.speed = self.minSpeed
# 			self.laneKeepingFlag = True

		elif self.part[2] is True:
			print("Part 2")
# 			self.angle = 0
			self.laneKeepingFlag = True
			self.counter += 1

		elif self.part[3] is True:
			print("Part 3")
			self.angle += self.step_sub
			self.speed = self.minSpeed
			self.laneKeepingFlag = False

		elif self.part[4] is True:
			print("Part 4")
			self.angle += self.step_sub
			self.speed = self.minSpeed

		return self.minSpeed, self.angle, overtake_flag, self.laneKeepingFlag
