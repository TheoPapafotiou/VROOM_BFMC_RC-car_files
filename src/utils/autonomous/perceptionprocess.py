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
from src.utils.autonomous.sign_detection       import SignDetection
from src.utils.autonomous.ped_detection        import PedestrianDetection
from src.utils.autonomous.shapes_detection     import ShapesDetection
from src.utils.autonomous.Line                 import Line
from src.utils.autonomous.Mask                 import Mask
from src.utils.autonomous.HelperFunctions      import HelperFunctions as hf
from src.utils.autonomous.LaneKeeping          import LaneKeeping as lk
from src.utils.autonomous.tracker              import Tracker


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
        self.signDet = SignDetection()
        self.pedDet = PedestrianDetection()
        self.tracker = cv2.TrackerMOSSE_create()    # high speed, low accuracy
        #self.tracker = cv2.TrackerCSRT_create()      # low speed, high accuracy
        #self.shapesDet = ShapesDetection()
        #self.port       =   2244
        #self.serverIp   =   '0.0.0.0'
        
        self.imgSize    = (480,640,3)
        self.imgHeight = 480
        self.imgWidth = 640
        self.countFrames = 0
        self.countFours = 0
        self.speed = 0.5
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
        readTh = Thread(name = 'PhotoReceiving',target = self._read_stream, args= (self.inPs[0], ))
        self.threads.append(readTh)
        """Initialize the read thread to send the edited video.
        """
        #sendTh = Thread(name = 'PhotoSending',target = self._read_stream, args= (self.inPs[0], self.outPs[0]))
        #self.threads.append(sendTh)
    
    def drawBox(img, bbox):
        x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
        cv2.rectangle(img, (x,y), ((x+w),(y+h)), (255, 0, 255), 3, 1)
        
    # ===================================== READ STREAM ==================================
    def _read_stream(self, inP):
        """Read the image from input stream, decode it and show it.

        Parameters
        ----------
        outPs : list(Pipe)
            output pipes (not used at the moment)
        """
        
        print('Start showing the photo')
        
        while True:
            try:
                stamps, img = inP.recv()
                self.countFrames+=1
                
                # ----------------------- read image -----------------------
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                
                #if self.countFrames == 1:
                #    bbox = cv2.selectROI("Tracking",img,False)
                #
                #    self.tracker.init(img, bbox)
                
                # ----------------------edit image -----------------------
                #self.shapesDet.getContours(img, imgGray)
                if self.countFours == 8:
                    self.countFours = 0
                if self.countFrames%20 <= 1:
                    self.countFours+=1
                    #self.signDet.detectSign(img, self.imgHeight, self.imgWidth, self.countFours)
                
                self.curr_steering_angle = lk.lane_keeping(img, self.speed, self.curr_steering_angle)
                #if self.countFrames%10 == 0:
                    #print("I'm also here")
                    #cv2.imwrite(str(stamps[0])+"_stop.jpg", img)
                    
                    
                    #timer = cv2.getTickCount()
                    #success, bbox = self.tracker.update(img)
                    #bbox_save1 = bbox[0]/640
                    #bbox_save2 = bbox[1]/480
                    #bbox_save3 = bbox[2]/640
                    #bbox_save4 = bbox[3]/480
                    #print(bbox[0])
                    #print(bbox_save1)
                    #if bbox[0] != 0 and bbox[1] != 0:
                    #    cv2.imwrite(str(stamps[0])+".jpg", img)
                    #
                    #if success:
                    #    drawBox(img, bbox)
                    #
                    #else:
                    #    cv2.putText(img, "Lost", (75,75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 3)
                    #
                    #fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
                    #cv2.putText(img, str(int(fps)), (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 3)
                    #cv2.putText(img, "Tracking", (75, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 3)
                    #cv2.imshow("Tracking" , img)
                
                
                # ----------------------- send results (image, perception) -------------------
                #cv2.imshow("image", img)
                #retval, buffer = cv2.imencode('.jpg', img)
                #img_as_text = base64.b64encode(buffer)
                perception_results = [self.curr_steering_angle]
                self.outPs[0].send([[stamps], img])
                self.outPs[1].send(perception_results)
                
            except:
                pass