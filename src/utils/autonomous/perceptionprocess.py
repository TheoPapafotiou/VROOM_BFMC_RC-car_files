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
#from src.utils.autonomous.sign_detection       import SignDetection
from src.utils.autonomous.shapes_detection     import ShapesDetection
from src.utils.autonomous.Line                 import Line
from src.utils.autonomous.Mask                 import Mask
from src.utils.autonomous.HelperFunctions      import HelperFunctions as hf
from src.utils.autonomous.LaneKeeping          import LaneKeeping as lk
from src.utils.autonomous.LaneKeepingReloaded  import LaneKeepingReloaded


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
        self.lane_keeping = LaneKeepingReloaded(640, 480)
        #self.tracker = cv2.TrackerMOSSE_create()    # high speed, low accuracy
        #self.tracker = cv2.TrackerCSRT_create()      # low speed, high accuracy
        #self.shapesDet = ShapesDetection()
        #self.port       =   2244
        #self.serverIp   =   '0.0.0.0'
        
        self.imgSize    = (480,640,3)
        self.imgHeight = 480
        self.imgWidth = 640
        self.img_sign = np.zeros((640, 480))
        self.countFrames = 0
        self.countFours = 0
        self.speed = 0.2
        self.intersection_navigation = False
        self.found_intersection = False
        self.curr_steering_angle = 90
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
                stamps, img = self.inPs[0].recv()
                self.countFrames+=1
                
                # ----------------------- read image -----------------------
                #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                img_dims = img[:,:,0].shape
                mask = Mask(4, img_dims)
                mask.set_polygon(np.array([[0,460], [640,460], [546,155], [78, 155]]))
                processed_img = hf.image_processing(img)
                masked_img = mask.apply_to_img(processed_img)
                #Process frame -END-
                
                # ----------------------detect sign in image -----------------------
                if self.countFrames%20 == 1:
                    print("Frame sent")
                    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    self.outPs[2].send([[stamps], img_bgr])
                    #label, confidence = self.signDet.detectSign(img, self.imgHeight, self.imgWidth)
                
                if self.countFrames%20 == 0:
                    stamps, self.img_sign = self.inPs[1].recv()

                #Detect lines -START-
                lane_lines = hf.detect_lane(masked_img)
                #Detect lines -END-
                
                #Detect intersections and distance to them -START-
                line_segments = hf.vector_to_lines(hf.detect_line_segments(masked_img))
                horizontal_line = hf.horizontal_line_detector(img, line_segments)
                check_for_intersection = False
                if(horizontal_line != None):
                    distance, detected_hor_line = hf.distance2intersection(horizontal_line, img)
                    #print(distance)
                    if distance < 80:
                        check_for_intersection = True
                        found_intersection = True
                        #speed = 0
                
                #if(self.intersection_navigation is False or len(lane_lines) == 2):
                    #print("LINE#: ",len(lane_lines))
                    #speed = start_speed
                    #self.curr_steering_angle = lk.lane_keeping(img, lane_lines, self.speed, self.curr_steering_angle, masked_img)
                self.curr_steering_angle, both_lanes, lane_frame = self.lane_keeping.lane_keeping_pipeline(img)
                    #self.curr_steering_angle /= 2
                                
                #DEBUG: Various helping windows -START-
                #hough_img = hf.get_hough_img(img, lane_lines) #Makes image with single lines on top
                #heading_img = hf.display_heading_line(hough_img, self.curr_steering_angle) #Makes image with heading line on top
                
                
                # ----------------------- send results (image, perception) -------------------
                perception_results = [self.curr_steering_angle]
                self.outPs[0].send([[stamps], lane_frame])
                self.outPs[1].send(perception_results)
                
            except:
                pass