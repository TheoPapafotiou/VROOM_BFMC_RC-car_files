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

import cv2
from threading import Thread

import multiprocessing
from multiprocessing import Process,Event

from src.utils.templates.workerprocess         import WorkerProcess
from src.utils.autonomous.sign_detection       import SignDetection
from src.utils.autonomous.ped_detection        import PedestrianDetection

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
        #self.port       =   2244
        #self.serverIp   =   '0.0.0.0'

        self.imgSize    = (480,640,3)
        self.imgHeight = 480
        self.imgWidth = 640
        self.countFrames = 0
        self.countFours = 0
        print("\nI am an initializer\n")
    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializers and start the threads.
        """
        super(PerceptionProcess,self).run()

    """
    # ===================================== INIT SOCKET ==================================
    def _init_socket(self):
        #Initialize the socket.
        
        self.server_socket = socket.socket(
                                    family  = socket.AF_INET, 
                                    type    = socket.SOCK_DGRAM
                                )
        self.server_socket.bind((self.serverIp, self.port))
        
        self.server_socket = socket.socket()
        self.server_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.server_socket.bind((self.serverIp, self.port))

        self.server_socket.listen(0)
        self.connection = self.server_socket.accept()[0].makefile('rb')
        
    """

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
                stamps, image = inP.recv()
                self.countFrames+=1
                #cv2.imwrite('color_img.jpg', image)
                
                # ----------------------- read image -----------------------
                img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                
                # ----------------------edit image -----------------------
                if self.countFours == 8:
                    self.countFours = 0
                if self.countFrames%2 <= 1:
                    self.countFours+=1
                    #self.signDet.detectSign(img, self.imgHeight, self.imgWidth, self.countFours)
                

                # ----------------------- show images -------------------
                cv2.imshow("image", img)
                perception_results = ["The perception has magnificent results"]
                #for outP in self.outPs:
                #    outP.send(perception_results)
                #print(img)
                self.outPs[1].send(perception_results)
                self.outPs[0].send(perception_results)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break              
                
            except:
                pass
        """
        finally:
            self.connection.close()
            self.server_socket.close()
        """