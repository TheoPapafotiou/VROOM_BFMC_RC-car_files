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


class SignDetectionProcess(WorkerProcess):
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
        super(SignDetectionProcess,self).__init__(inPs, outPs)
        self.signDet = SignDetection()
        
        self.imgSize    = (480,640,3)
        self.imgHeight = 480
        self.imgWidth = 640
        self.img = np.zeros((640, 480))
        self.countFrames = 0
        self.countFours = 0

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializers and start the threads.
        """
        super(SignDetectionProcess,self).run()

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the signDetection thread to receive the video.
        """
        signTh = Thread(name = 'SignDetectionThread',target = self._haha)
        self.threads.append(signTh)
        
    # ===================================== DETECT SIGN =================================
    def _haha(self):
        print("Starting Sign-detection thread");
        
        while True:
            try:
                stamps, img = self.inPs[0].recv()
                print("Frame received")
                label, confidence = self.signDet.detectSign(img, self.imgHeight, self.imgWidth)
                print("I'm done")
                try:
                    for outP in self.outPs:
                        outP.send([[stamps], [label]])
                        print("Frame with sign sent")
                    
                except Exception as e:
                    print(e)
                        
            except:
                pass                
            
        print("Exiting Sign-detection thread");
