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
from src.utils.autonomous.vehicleHandler       import VehicleHandler


class VehicleDetectionProcess(WorkerProcess):
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
        super(VehicleDetectionProcess,self).__init__(inPs, outPs)
        self.vehDet = VehicleHandler()
        
        self.imgSize = (480,640,3)
        self.img = np.zeros((640, 480))

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializers and start the threads.
        """
        super(VehicleDetectionProcess,self).run()

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the vehicleDetection thread to receive the video.
        """
        vehTh = Thread(name = 'VehicleDetectionThread',target = self._haha2)
        self.threads.append(vehTh)
        
    # ===================================== DETECT VEHICLE =================================
    def _haha2(self):
        print("Starting Vehicle-detection thread")
        detected_vehicle = False
        
        while True:
            try:
                stamps, img = self.inPs[0].recv()
                print("Frame received2")
                detected_vehicle = self.vehDet.detect_vehicle(img)
                print("I'm done2")
                try:
                    for outP in self.outPs:
                        outP.send([[stamps], img])
                        print("Frame with vehicle sent")
                    
                except Exception as e:
                    print(e)
                        
            except:
                pass                
            
        print("Exiting Vehicle-detection thread");
