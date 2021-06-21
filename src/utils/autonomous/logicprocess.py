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
from multiprocessing import Value
from multiprocessing import Pipe


from src.utils.templates.workerprocess          import WorkerProcess

class LogicProcess(WorkerProcess):
    # ===================================== INIT==========================================
    #reset = False
    def __init__(self, inPs, outPs):
        """Logic Process. This should run on RPi4.

        """
        super(LogicProcess,self).__init__(inPs, outPs)

        # Can be change to a multithread.Queue.
        self.lisBrR, self.lisBrS = Pipe(duplex=False)

        self.enPID = True
        self.reset = Value("i", 0)
        self.port      =  12244
        self.serverIp  = '0.0.0.0'
        self.count = 0
        self.countFrames = 0
        self.Kp = 0.0
        self.Kd = 0.0
        self.Ki = 0.0
        self.speed = 0.0
        self.current_angle = 0.0

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
        start_thread = time.time()

        while self.reset.value == 0:
#             time.sleep(0.001)
            self.countFrames += 1
            
            if self.countFrames == 31:
                self.countFrames = 1
            start = time.time()
            
            if self.countFrames % 1 == 0:
                perception_results, start_time_command = inP.recv()
                self.current_angle = round(perception_results[0]*1.0, 1)
                self.speed = perception_results[1]*1.0
            
            end_time_command = time.time()
            """
            This is the place where we will decide for the command
            """
            print("^"*20)
            print("Speed: ", self.speed, "  Angle: ", self.current_angle)
            print("^"*20)

            #commandSPID = {'action': 'PIDS', 'kp': self.Kp, 'ki': self.Ki, 'kd': self.Kd, 'tf': 1.0}
#             commandSPID = {'action': 'PIDS', 'kp': 0.0005, 'ki': 0.001, 'kd': 0.00222, 'tf': 0.04}
            commandSPID = {'action': 'PIDS', 'kp': 0.008, 'ki': 0.01, 'kd': 0.00222, 'tf': 0.04}

            commandP = {'action': 'PIDA','activate': True}
            commandM = {'action': 'MCTL', 'speed': self.speed, 'steerAngle': self.current_angle}
            commandB = {'action': 'BRAK', 'steerAngle': self.current_angle}
            
            try:
                for outP in self.outPs:
                    #start = time.time()
                    if(self.count == 0):
                        outP.send(commandP)
                        outP.send(commandSPID)
                        self.count += 1
                    else:
                        outP.send(commandM)
                    #print("Duration of the command to be send: ", time.time() - start)
                    #print("\nTotal duration of logic: ", time.time() - start, "\n")

            except Exception as e:
                print(e)
        
        print("This is a reset: ", self.reset.value)
        speed = 0.0
        current_angle = 0.0
        print("Reset variables to 0.0")
        command = {'action': 'MCTL', 'speed': speed, 'steerAngle': current_angle}
        for outP in self.outPs:
            print("I will send the commands my lord")
            outP.send(command)