import time
import socket
import struct
import io
import base64

import cv2
import numpy as np
import imutils
from imutils.object_detection import non_max_suppression

from threading import Thread

import multiprocessing
from multiprocessing import Process,Event


from src.utils.templates.workerprocess         import WorkerProcess
from src.utils.autonomous.pedestrianHandler    import PedestrianHandler

class PedestrianDetectionProcess(WorkerProcess):
    # ===================================== INIT =========================================
    def __init__(self, inPs, outPs):
        """
        Parameters
        ----------
        inPs : list(Pipe)
            List of input pipes
        outPs : list(Pipe)
            List of output pipes
        """
        super(PedestrianDetectionProcess,self).__init__(inPs, outPs)
        self.pedDet = PedestrianHandler()
        
        self.img = np.zeros((640, 480))

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializers and start the threads.
        """
        super(PedestrianDetectionProcess,self).run()

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the pedestrianDetection thread to receive the video.
        """
        pedTh = Thread(name = 'PedestrianDetectionThread',target = self.pedestrian_detector)
        self.threads.append(pedTh)
        
    # ===================================== DETECT PEDESTRIAN =============================
    def pedestrian_detector(self):
        print("Starting Pedestrian-detection thread")
        detected_pedestrian = False
        
        while True:
            try:
                start = time.time()
                stamps, img = self.inPs[0].recv()
                try:
                    detected_pedestrian = self.pedDet.detectPedestrian(img)
                except:
                    print("\n\n")
                    print("Error in Pedestrian Detection PROCESS")
                    print("\n\n")
                print("PedDetection duration: ", time.time() - start)
                try:
                    for outP in self.outPs:
                        outP.send([[stamps], img])
                        print("Frame with pedestrian sent")
                    
                except Exception as e:
                    print(e)
                        
            except:
                pass                
            
        print("Exiting Pedestrian-detection thread");