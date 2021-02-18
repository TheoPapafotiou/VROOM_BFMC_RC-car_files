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
import json
import socket
import cv2
import time
import random

from threading       import  Thread
from multiprocessing import  Pipe

from src.utils.templates.workerprocess          import WorkerProcess

class LogicProcess(WorkerProcess):
    # ===================================== INIT==========================================
    def __init__(self, inPs, outPs):
        """Logic Process. This should run on RPi4.

        """
        super(LogicProcess,self).__init__(inPs, outPs)

        # Can be change to a multithread.Queue.
        self.lisBrR, self.lisBrS = Pipe(duplex=False)

        self.reset     = False
        self.port      =  12244
        self.serverIp  = '0.0.0.0'

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads
        """
        self._init_socket()
        super(LogicProcess,self).run()
        
    # ===================================== INIT SOCKET ==================================
    def _init_socket(self):
        """Initialize the socket for communication with remote client.
        """
        self.client_socket = socket.socket(
                                family  = socket.AF_INET,
                                type    = socket.SOCK_DGRAM
                            )
    #=============================== INIT THREADS ======================================
    def _init_threads(self):
        """Initialize the read thread to transmite the received messages to other processes. 
        """
        sendTh = Thread(name='SendCommand',target = self._send_command_thread, args = (self.inPs[0],  ))
        self.threads.append(sendTh)
    
    # ===================================== SEND COMMAND =================================
    def _send_command_thread(self, inP):
        """Transmite the command"""
        while self.reset is False:
            time.sleep(1)
            perception_results = inP.recv()
            """
            This is the place where we will decide for the command
            """
            current_angle = perception_results[0]*1.0 - 90.0
            if(abs(current_angle - 90) >= 5):
                speed = 0.1
            else:
                speed = 1.0                
                
            print("Speed: ", speed, "  Angle: ", current_angle)
            command = {'action': 'MCTL', 'speed': speed, 'steerAngle': current_angle}
            try:
                for outP in self.outPs:
                    outP.send(command)
                
            except Exception as e:
                print(e)
        
        speed = 0.0
        current_angle = 0.0
        print("Reset variables to 0.0")
        command = {'action': 'MCTL', 'speed': speed, 'steerAngle': current_angle}
        for outP in self.outPs:
            outP.send(command)
