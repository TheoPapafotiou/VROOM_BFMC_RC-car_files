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
from src.utils.autonomous.LaneKeepingReloaded               import LaneKeepingReloaded
from src.utils.autonomous.MainFunctions        import MainFunctions as mf


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

        self.pedDet = PedestrianDetection()
        self.lane_keeping = LaneKeepingReloaded(self.imgWidth, self.imgHeight)

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializers and start the threads.
        """
        super(PerceptionProcess,self).run()

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the read thread to receive the video.
        """
        readTh = Thread(name = 'PerceptionProcess',target = self._process_img)
        self.threads.append(readTh)
    
    # ===================================== READ STREAM ==================================
    def _process_img(self):
        """Read the image from input stream, decode it and process it.

        Parameters
        ----------
        outPs : list(Pipe)
            output pipes (not used at the moment)
        """
                
        while True:
            try:
                ###### RECEIVE FRAME ######
                stamps, img = self.inPs[0].recv()
                self.countFrames+=1
                
                ###### PROCESS FRAME ######
                img_dims = img[:,:,0].shape
                mask = Mask(4, img_dims)
                mask.set_polygon(np.array([[0,460], [640,460], [546,155], [78, 155]]))
                processed_img = hf.image_processing(img)
                masked_img = mask.apply_to_img(processed_img)
                
                ###### SIGN DETECTION ######
                if self.countFrames%20 == 1:
                    print("Frame sent")
                    self.outPs[2].send([[stamps], img])
                
                if self.countFrames%20 == 0:
                    stamps, self.img_sign = self.inPs[1].recv()

                ###### DETECT LINES ######
                lane_lines = hf.detect_lane(masked_img)
                
                lane_keeping_angle, both_lanes = lane_keeping.lane_keeping_pipeline(img)
                self.speed, self.angle = mf.lane_keeping(self.speed, lane_keeping_angle)
                
                
                # ----------------------- send results (image, perception) -------------------
                perception_results = [self.angle, self.speed]
                self.outPs[0].send([[stamps], self.img_sign])
                self.outPs[1].send(perception_results)
                
            except:
                pass