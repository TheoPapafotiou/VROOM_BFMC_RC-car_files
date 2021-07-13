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
		self.overtakeTime = 0
		self.startTime = 0
		self.minSpeed = 0.12
		self.minDistance = 330
		self.laneWidth = 3.5
		self.safeDistance = 78.67
		self.safeDuration = 2.26
		self.currentX = 0
		self.currentY = 0
		self.targetX = 0
		self.targetY = 0

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

	def make_detour(self, overtake_flag, cur_time):
		if (self.overtakeTime < 2*self.safeDuration):
			print("Part 0")

			self.overtakeTime = cur_time - self.startTime
			self.angle = -1 * self.calculate_steering_angle(self.overtakeTime)
			print("Part 0 angle = ", self.angle, "\nOvertake Time =", self.overtakeTime)
            
		elif self.overtakeTime >= 2*self.safeDuration and self.overtakeTime < 3*self.safeDuration:
			print("Part 1")

			self.overtakeTime = cur_time - self.startTime
			self.laneKeepingFlag = True
			print("Overtake timee = ", self.overtakeTime)
		
		elif self.overtakeTime >= 3*self.safeDuration and self.overtakeTime < 4*self.safeDuration : #2.2 for straight & 2 for turns
			print("Part 2")

			self.overtakeTime = cur_time - self.startTime
			self.angle = self.calculate_steering_angle(self.overtakeTime - 3*self.safeDuration)
			self.laneKeepingFlag = False
			print("Overtake timee = ", self.overtakeTime)


		else:
			print("Part 3")
			self.startTime = 0
			#self.laneKeepingFlag = True
			self.speed = 0
			overtake_flag = False
			print("Overtake timee = ", self.overtakeTime)

			#self.overtakeFlag = False

		return self.speed, self.angle, overtake_flag, self.laneKeepingFlag

	def calculate_x(self, cur_time):
		x = 70*self.minSpeed * cur_time + ((70*self.minSpeed * self.safeDuration - self.safeDistance))*(10*((cur_time/self.safeDuration)**3) - 15*((cur_time/self.safeDuration)**4) + 6*((cur_time/self.safeDuration)**5))
		return x

	def calculate_y(self, cur_time):
		y = self.laneWidth + self.laneWidth*(10*((cur_time/self.safeDuration)**3) - 15*((cur_time/self.safeDuration)**4) + 6*((cur_time/self.safeDuration)**5))
		return y

	def calculate_steering_angle(self, cur_time):
		self.currentX = self.targetX
		self.currentY = self.targetY

		self.targetX = self.calculate_x(cur_time)
		self.targetY = self.calculate_y(cur_time)
		
		if(self.targetX - self.currentX == 0 ):
			return self.angle

		return math.atan2((self.targetY - self.currentY), (self.targetX - self.currentX))
