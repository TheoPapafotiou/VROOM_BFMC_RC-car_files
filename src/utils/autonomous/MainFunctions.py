#from src.utils.autonomous.gps_s import Gps
#from src.utils.autonomous.bno055_s import BNO055
#from src.utils.autonomous.camera_s import CameraHandler
#from src.utils.autonomous.controller_p import Controller
#from src.utils.autonomous.trafficlight_s import TLColor, TLLabel, TrafficLight
from src.utils.autonomous.objectDetection import ObjectDetection
from src.utils.autonomous.Line import Line
from src.utils.autonomous.Mask import Mask
from src.utils.autonomous.HelperFunctions import HelperFunctions as hf
#from src.utils.autonomous.PathPlanning import PathPlanning as pp
from imutils.object_detection import non_max_suppression
from src.utils.autonomous.pedestrianDetection import PedestrianDetection
from src.utils.autonomous.Parking import Parking
from src.utils.autonomous.LaneKeepingReloaded import LaneKeepingReloaded 
import src.utils.autonomous.utils as utils
#from src.utils.autonomous.Roundabout import Roundabout 
from src.utils.autonomous.vehicleHandler import VehicleHandler

import imutils
import os
import rospy
import cv2
from time import time
import numpy as np
import math

import sys
import networkx as nx

"""
A class where all the functions called by the main.py file are stored.
"""

class MainFunctions:

    def __init__(self):

        self.RedisC = RedisComm()

        self.ObjTrack = ObjectDetection()

        self.car = Controller()

        self.sem = TrafficLight()

        self.gps = Gps()

        self.bno = BNO055()

        self.pedTrack = PedestrianDetection()

        self.parking = Parking()

        self.speed = 0.5
        self.initial_angle = 0.0
        self.min_starting_speed = 0.5
        self.x, self.y, self.yaw = 0,0,0
        self.imgWidth = 640
        self.imgHeight = 480
        self.frame = np.zeros((self.imgWidth, self.imgHeight))
        self.sign_feed_limit = 20
        self.start_time = 0.0
        self.turn_type = 0
        self.inter_detect_limit = 60

        # Rolling average parameters
        self.rolling_average = []
        self.rolling_length = 1
        
    def start(self):
        speed = self.speed
        angle = self.initial_angle
        return speed, angle

    def pause(self):
        speed = 0
        angle = 0
        return speed, angle

    def path_planning(self, path, x, y, finish):
        path, reachedFinish = pp.update_path(path, x, y, finish)
        return path, reachedFinish

    def GPS(self):
        gps_data = self.gps.getGpsData()
        if(gps_data["coor"] != None):
            data = self.gps.getGpsData()
            x = gps_data["coor"][0].real
            y = gps_data["coor"][0].imag

        return x, y

    def lane_keeping(self, speed, angle):

        ###### ROLLING AVERAGE ON THE ANGLE ######
        self.rolling_average.append(angle)

        if len(self.rolling_average) > self.rolling_length:
            self.rolling_average.pop(0)

        angle = np.average(self.rolling_average)

        return speed, angle 

    def reduce_speed(self, speed, angle):
        speed /= 2
        return speed, angle

    def parking_horizontal(self, yaw_init, yaw, frame, flag):

        return self.parking.parking_horizontal(yaw_init, yaw, frame, flag)

    def parking_vertical(self, yaw_init, yaw, frame, flag):

        return self.parking.parking_vertical(yaw_init, yaw, frame, flag)

    def roundabout_navigation(self, roundabout, lane_keeping_angle, yaw, speed):
        
        if roundabout.isEnteringRoundabout():

            angle, stop = roundabout.enteringRoundabout(yaw)

            if stop:
                roundabout.stopEnterSequence()
                speed = self.start_speed
                roundabout.startTurn()

        if roundabout.isTurning():

            angle, laneKeepingFlag, destroy = roundabout.roundaboutTurns(yaw)
            if laneKeepingFlag:
                angle = lane_keeping_angle
            if destroy:
                roundabout = None

        return speed, angle, roundabout


    def inter_navigation(self, path, x, y, complete_path, intersection_navigation, target_node,  start_yaw, yaw, steer_factor, speed, start_time):
        angle, reached_target = pp.intersection_navigation(path, x, y, target_node, 
                                                        start_yaw, yaw, complete_path, steer_factor, speed, start_time)
        
        if(reached_target): 
            intersection_navigation = False
        speed = self.speed

        return speed, angle, intersection_navigation

    def reverse(self, speed, angle):
        speed = -self.min_starting_speed
        angle *= -1
        return speed, angle

    def increase_speed(self, speed, angle):
        speed *= 2
        if speed >= self.max_speed:
            speed = self.max_speed
        return speed, angle

    def overtake(self, complete_path, veh_frame, vehicle_detected, overtake_flag, yaw, lane_keeping_angle):

        dotted = VehicleHandler.check_dotted_line(pp.G, complete_path[0], complete_path[1])
        
        if vehicle_detected is True and overtake_flag is False:
            VehicleHandler.distance_to_vehicle(veh_frame)
            speed, overtake_flag = VehicleHandler.react_to_vehicle(dotted)
            
            if overtake_flag is True:
                VehicleHandler.startTime = rospy.get_time()

        if overtake_flag is True:
            speed, steering, overtake_flag, lane_keeping_flag= VehicleHandler.make_detour(frame, overtake_flag, rospy.get_time())

            if(lane_keeping_flag is True):
                steering = lane_keeping_angle
                VehicleHandler.angle = steering
            
        else:
            speed = speed_init
            steering = lane_keeping_angle

        return speed, steering, overtake_flag
    
    def tailing(self, frame, angle, dist_from_vehicle):
        speed /= (self.imgHeight/dist_from_vehicle)

        return speed, angle
    
    def sign_detection(self, frame, countFrames):

        #Publish image
        if countFrames%self.sign_feed_limit == 0:
            self.RedisC.publish("simulation-channel", frame)
        
        #Detect Sign
        message = self.RedisC.getMessage(1)
        for i in range (0, 1):
            if message is None:
                message = self.RedisC.getMessage(1)
            else:
                break
        if message is not None:
            result = self.RedisC.translate(message)
            if result is not 0:        
                #Take label
                label = self.RedisC.getMessage(2)
                distance = self.RedisC.getMessage(3)
        
                return label, distance, result
        return None, None, None

    def pedestrian_detection(self,imgPed):
        pedDetect = self.pedTrack.detectPedestrian(imgPed)

        return pedDetect, imgPed

    def intersection_detection(self, path, reached_finish, masked_img, frame, intersection_navigation, found_intersection, start_yaw, yaw, target_node):

        line_segments = hf.vector_to_lines(hf.detect_line_segments(masked_img))
        horizontal_line = hf.horizontal_line_detector(frame, line_segments)
        check_for_intersection = False
        if(horizontal_line is not None and intersection_navigation == False):
            distance, detected_hor_line = hf.distance2intersection(horizontal_line, frame)
            if distance < self.inter_detect_limit:
                if pp.check_intersection(path):
                    check_for_intersection = True
                    found_intersection = True
                    target_node, reached_finish = pp.find_target(path)
                    start_yaw = yaw
                    self.start_time = time()
                elif pp.check_roundabout(path):
                    speed = 0.2
                    self.start_time = time()
                    ra_entry = pp.find_roundabout_entry(path)
                    ra_exit = pp.find_roundabout_exit(path)
                    roundabout = Roundabout(self.start_time, start=ra_entry, end=ra_exit)

        return found_intersection, start_yaw, target_node, reached_finish, self.start_time
