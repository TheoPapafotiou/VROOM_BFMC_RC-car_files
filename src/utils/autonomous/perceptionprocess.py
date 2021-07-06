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
from src.utils.autonomous.Parking              import Parking
from src.hardware.BNOHandler.BNOhandler        import BNOhandler


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
        #self.signDet = SignDetection()
        self.pedDet = PedestrianDetection()
        self.park = Parking()
        self.lane_keeping = LaneKeepingReloaded(640, 480)
        
        self.imgHeight = 480
        self.imgWidth = 640
        self.img_sign = np.zeros((self.imgWidth, self.imgHeight))
        self.countFrames = 0
        self.speed = 0.0
        self.intersection_navigation = False
        self.found_intersection = False
        self.curr_steering_angle = 0
        
        self.sign_detected = "None"

        ### Parking params ###
        self.parking_ready = False
        self.parking_initiated = False
        self.parking_flag_1 = False
        self.parking_flag_2 = False
        self.parking_type = "False"
        self.norm_factor = 300/480
        self.count_steps = 0
        self.max_count_steps = 0
        self.yaw_init = 0
        
        global IMU
        signal.signal(signal.SIGTERM, self.exit_handler)
        IMU = BNOhandler()

        #self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        #self.out = cv2.VideoWriter('lab_test_video.avi',self.fourcc, 30.0 , (self.imgWidth,self.imgHeight)) 

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
        

        IMU.start()
        while True:
            try:
                #print("\nPerception Process")
                start = time.time()
                self.countFrames += 1
                self.count_steps += 1

                yaw = IMU.yaw
                print("Yaw is: ", yaw)

                stamps, img = self.inPs[0].recv()
                #print("Time for taking the perception image: ", time.time() - start)
                
                # ----------------------detect sign in image -----------------------
                start = time.time()
    #                 if self.countFrames%20 == 1:
    #                     img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    #                     self.outPs[2].send([[stamps], img_bgr])
    #                 
    #                 elif self.countFrames%20 == 0:
    #                     stamps, self.img_sign, self.sign_detected = self.inPs[1].recv()
                   #print("Sign detected: ", self.sign_detected)
                #print("Sign detection duration: ", time.time() - start)
                
                start = time.time()
                self.speed = 0.1
                
                #### LANE KEEPING ####
                self.curr_steering_angle, both_lanes, lane_frame = self.lane_keeping.lane_keeping_pipeline(img)
                
                #print("Lane Keeping duration: ", time.time() - start)
                
                if self.countFrames == 2:
                    self.sign_detected = 'ParkingSpot'
                if self.countFrames == 3:
                    self.sign_detected = 'None'
                    
                #### PARKING ####
                if self.sign_detected == 'ParkingSpot' and self.parking_ready is False and self.parking_initiated is False:
                    
                    # if x < 3 and y < 2.3:
                    #     self.parking_type = "vertical"
                    # else:
                    #     self.parking_type = "horizontal"
                    self.parking_type = "vertical"

                    self.max_count_steps = 3#11#distance*self.norm_factor
                    self.count_steps = 0
                    self.parking_ready = True

                if self.count_steps == self.max_count_steps and self.parking_ready is True:
                    self.yaw_init = 0.0
                    print("Yaw init is: ", self.yaw_init)
                    self.parking_initiated = True
                    self.parking_ready = False

                if self.parking_initiated is True:
                    if parking_type == 'vertical':
                        self.speed, self.curr_steering_angle, self.parking_initiated = self.park.parking_vertical(self.yaw_init, yaw, self.parking_initiated)
                    else:
                        self.speed, self.curr_steering_angle, self.parking_initiated = park.parking_horizontal(self.yaw_init, yaw, img, self.parking_initiated)
                    print(self.speed, self.curr_steering_angle)

                #### NORMALIZE ANGLE ####
                if self.curr_steering_angle >= 25:
                    self.curr_steering_angle = 23
                if self.curr_steering_angle <= -25:
                    self.curr_steering_angle = -23
                
                #### SEND RESULTS (image, perception) ####
                perception_results = [self.curr_steering_angle, self.speed]
                self.outPs[0].send([[stamps], img])
                start_time_command = time.time()
                self.outPs[1].send([perception_results, start_time_command])
                
                #print("\nTotal duration of perception: ", time.time() - start, "\n")
            except:
                pass