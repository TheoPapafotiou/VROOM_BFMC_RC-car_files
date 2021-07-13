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
sys.path.append('.')

import time
import socket
import struct
import numpy as np
from PIL import Image
import io
import base64

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
from src.utils.autonomous.Overtake             import OvertakeProcedure


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
        self.overtake = OvertakeProcedure()
        
        self.imgHeight = 480
        self.imgWidth = 640
        self.speed = 0.0
        self.intersection_navigation = False
        self.found_intersection = False
        self.curr_steering_angle = 0
        self.angle_factor = 23.0/90

        ## Overtake Parameters ##

        self.vehicle_detected = True
        self.overtake_flag = False 
        self.dotted_line = True
        self.lane_keeping_flag = False
        self.lane_keeping_angle = 0.00

        
    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializers and start the threads.
        """
        super(PerceptionProcess,self).run()

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the                if self.sign_detected == 'ParkingSpot' and self.parking_ready is False and self.parking_initiated is False:if self.sign_detected == 'ParkingSpot' and self.parking_ready is False and self.parking_initiated is False:read thread to receive the video.
        """
        readTh = Thread(name = 'PhotoReceiving',target = self._read_stream)
        self.threads.append(readTh)
    
    # ===================================== READ STREAM ==================================
    def _read_stream(self):
        """Read the image from input stream, decode it and show it.

        Parameters
        ----------
        outPs : list(Pipe)
            output pipes (not used at the moment)
        """
        
        print('Start showing the photo')

        while True:
            try:
                start = time.time()
                stamps, img = self.inPs[0].recv()
                            
                start = time.time()
                
                self.speed = 0.08
                img_lane = cv2.resize(img, (320,240), interpolation=cv2.INTER_AREA)

                # ----------------------- Lane keeping process -------------------

                self.lane_keeping_angle, lane_frame = self.lane_keeping.lane_keeping_pipeline(img_lane)
                self.lane_keeping_angle *= self.angle_factor
                    
                if abs(self.lane_keeping_angle) > 12:
                    self.speed = 0.13

                # ----------------------- Overtake Procedure -------------------

                #TODO: Adjust this once path planning is added.

                if self.vehicle_detected is True and self.overtake_flag is False: 

                    # self.dotted_line = self.overtake.check_dotted_line(graph, source, target)
                    # self.overtake.distance_to_vehicle(img, bbox)
                    # self.speed, self.overtake_flag = self.overtake.react_to_vehicle(self.dotted_line)
                    
                    self.overtake_flag = True          # This is only for testing reasons 
            
                    if self.overtake_flag is True:
                        self.overtake.startTime = time.time()
                        print("Overtake start time = ", self.overtake.startTime)
                        
                print("Overtake flag = ", self.overtake_flag, "Vehicle detected fag = ", self.vehicle_detected)
                
                if self.overtake_flag is True:
                    self.speed, self.curr_steering_angle, self.overtake_flag, self.lane_keeping_flag = self.overtake.make_detour(self.overtake_flag, time.time())

                    if(self.lane_keeping_flag is True):
                        self.curr_steering_angle = self.lane_keeping_angle
                
                else:
                    self.curr_steering_angle = self.lane_keeping_angle
                self.speed = 0.0
                
                if self.curr_steering_angle > 23.0:
                    self.curr_steering_angle = 23.0
                
                elif self.curr_steering_angle < -23.0:
                    self.curr_steering_angle = -23.0
                
                print("Lane keeping  angle = ", self.lane_keeping_angle)
                print("Angle = ", self.curr_steering_angle, "\nSpeed =", self.speed)
                # ----------------------- send results (image, perception) -------------------
                perception_results = [self.curr_steering_angle, self.speed]
                stamp = time.time()
                self.outPs[0].send([[stamp], lane_frame])
                self.outPs[1].send([perception_results, stamp])
                
                #print("\nTotal duration of perception: ", time.time() - start, "\n")
            except:
                raise Exception("Maybe error in lane keeping")
                pass