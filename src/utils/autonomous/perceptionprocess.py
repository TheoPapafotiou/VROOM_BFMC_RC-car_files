# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
import sys

#from networkx.generators.classic import complete_graph
sys.path.append('.')

import time
import socket
import struct
import numpy as np
from PIL import Image
import io
import base64
import signal

import cv2
from threading import Thread

import multiprocessing
from multiprocessing import Process,Event

from src.utils.templates.workerprocess         import WorkerProcess
from src.utils.autonomous.ped_detection        import PedestrianDetection
from src.utils.autonomous.shapes_detection     import ShapesDetection
from src.utils.autonomous.Line                 import Line
from src.utils.autonomous.Mask                 import Mask
from src.utils.autonomous.HelperFunctions      import HelperFunctions as hf
from src.utils.autonomous.LaneKeeping          import LaneKeeping as lk
from src.utils.autonomous.LaneKeepingReloaded  import LaneKeepingReloaded
from src.data.carstracker.carstracker          import gps_listener
from src.hardware.BNOHandler.BNOhandler        import BNOhandler
from src.utils.autonomous.PathPlanning         import PathPlanning as pp

class PerceptionProcess(WorkerProcess):
    # ===================================== INIT =========================================
    def __init__(self, inPs, outPs):
        """Process used for debugging. It receives the images from the raspberry and
        duplicates the perception pipline that is running on the raspberry.

        Parameters
        ----------
        inPs : list(Pipe)
            List of input pipes
        outPs : list(Pipe)
            List of output pipes
        """
        super(PerceptionProcess,self).__init__(inPs, outPs)
        self.lane_keeping = LaneKeepingReloaded(320, 240)
        
        self.imgHeight = 480
        self.imgWidth = 640
        self.img_sign = np.zeros((self.imgWidth, self.imgHeight))
        self.label = None
        self.countFrames = 0
        self.speed = 0.0
        self.intersection_navigation = False
        self.found_intersection = False
        self.curr_steering_angle = 0
        self.angle_factor = 23.0/90


        #Intersection variables
        self.target_node = None
        self.reached_target = False
        source, finish = '38', '48'

        self.complete_path = pp.shortest_path(source, finish)
        self.path = pp.remove_central_nodes(self.complete_path)

        self.start_yaw = 0
        self.start_time = 0


        self.polygon_array = np.array([[0,460], [640,460], [546,260], [78, 260]])
        self.hor_detect_limit = 160
        self.horizontal_line = False

        global GPS
        signal.signal(signal.SIGTERM, self.exit_GPS_handler)
        GPS = gps_listener()

        global IMU
        signal.signal(signal.SIGTERM, self.exit_handler)
        IMU = BNOhandler()
        
    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializers and start the threads.
        """
        super(PerceptionProcess,self).run()

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the read thread to receive the video.
        """
        readTh = Thread(name = 'PhotoReceiving',target = self._read_stream)
        self.threads.append(readTh)

    # ==================================== GPS HANDLER EXIT ==============================
    def exit_GPS_handler(self, signum, frame):
        GPS.stop()
        GPS.join()
        sys.exit(0)

    # ==================================== BNO HANDLER EXIT ==============================
    def exit_handler(self, signum, frame):
        IMU.stop()
        IMU.join()
        sys.exit(0)
    
    # ===================================== READ STREAM ==================================
    def _read_stream(self):
        """Read the image from input stream, decode it and show it.

        Parameters
        ----------
        outPs : list(Pipe)
            output pipes (not used at the moment)
        """
        
        print('Start showing the photo')

        GPS.start()
        IMU.start()
        while True:
            try:
                start = time.time()
                self.countFrames += 1
                stamps, img = self.inPs[0].recv()
#                 print("&"*20)
#                 print("Time for the transfer of the perception image: ", time.time() - stamps[0])
#                 print("&"*20) 
                X = GPS.X
                Y = GPS.Y
                #print("Coordinates are: ", X, ", ", Y)

                yaw = IMU.yaw
                yaw = 180 - yaw # so that the actual car fits with the simulation coordinate system
                if yaw > 180:
                    yaw = yaw - 360 # [0 || 180] - [0 || -180]
                #print("Yaw is: ", yaw)
                
                if self.label is None:
                    self.img_sign = img

                # ---------------------process frame ------------------------------
                img_dims = img[:,:,0].shape
                mask = Mask(4, img_dims)
                mask.set_polygon_points(self.polygon_array)
                processed_img = hf.image_processing(img)
                masked_img = mask.apply_to_img(processed_img)

                # ----------------------detect horizontal line ---------------------
                self.horizontal_line = False
                line_segments = hf.vector_to_lines(hf.detect_line_segments(masked_img))
                horizontal_line = hf.horizontal_line_detector(img, line_segments)
                distance_hor, detected_hor_line, self.horizontal_img = hf.distance2intersection(horizontal_line, img)
                print("Distance_hor: ", distance_hor)
                if distance_hor < self.hor_detect_limit:
                    self.horizontal_line = True
                
                # ----------------------detect sign in image -----------------------
#                 start = time.time()
#                 if self.countFrames%10 == 1:
#                     img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
#                     self.outPs[2].send([[stamps], img_bgr])
#                 
#                 elif self.countFrames%10 == 0:
#                    stamps, self.img_sign, self.label = self.inPs[1].recv()
#                    print("\n===========\n", self.label, "\n===========\n")
#                 print("Sign Detection Duration: ", time.time() - start)
                
                start = time.time()
                
#                 if self.label is not None:
#                     self.speed = 0.0
                self.speed = 0.08

                img_lane = cv2.resize(img, (320,240), interpolation=cv2.INTER_AREA)
                self.lane_keeping_angle, lane_frame = self.lane_keeping.lane_keeping_pipeline(img_lane)
             


                # ---------------------- intersection navigation -----------------------
                
                #Check if at intersection
                if(self.horizontal_line and self.intersection_navigation == False):


                    #NA MPEI ELEGXOS AN EXOYME INTERSECTION NODE
                    self.found_intersection = True
                    self.target_node, self.reached_target = pp.find_target(self.path)

                    self.start_yaw = yaw
                    self.start_time = time.time()

                print("Found Intersection: ", self.found_intersection)
                print("Path ", self.path)
                if(self.intersection_navigation == False):
                    self.curr_steering_angle = self.lane_keeping_angle


                if(self.found_intersection or self.intersection_navigation):
                    if(self.intersection_navigation == False):
                        self.intersection_navigation = True
                        self.curr_steering_angle, self.reached_target = pp.intersection_navigation(self.path, X, Y, self.target_node, self.start_yaw, yaw, self.complete_path,  self.speed, self.start_time)
                    
                        self.found_intersection = False
                    else:
                        self.curr_steering_angle, self.reached_target = pp.intersection_navigation(self.path, X, Y, self.target_node, self.start_yaw, yaw, self.complete_path,  self.speed, self.start_time)
                    
                    if(self.reached_target):
                            self.intersection_navigation = False
                    

                # ---------------------- factor angle -----------------------
                self.curr_steering_angle *= self.angle_factor
                if abs(self.curr_steering_angle) > 12:
                    self.speed = 0.13
                
#                 print("Lane Keeping duration: ", time.time() - start)
                
                # ----------------------- send results (image, perception) -------------------
                perception_results = [self.curr_steering_angle, self.speed]
                stamp = time.time()
                self.outPs[0].send([[stamp], img_lane])
                self.outPs[1].send([perception_results, stamp])
                
                #print("\nTotal duration of perception: ", time.time() - start, "\n")
            except:
                raise Exception("Maybe error in lane keeping")
                pass