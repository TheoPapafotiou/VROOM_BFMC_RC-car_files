# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC orginazers
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
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#========================================================================
# SCRIPT USED FOR WIRING ALL COMPONENTS
#========================================================================
import sys
sys.path.append('.')

import time
import signal
from multiprocessing import Pipe, Process, Event

# hardware imports
from src.hardware.camera.cameraprocess               import CameraProcess
from src.hardware.serialhandler.serialhandler        import SerialHandler

# data imports
# from src.data.consumer.consumerprocess             import Consumer

# utility imports
from src.utils.camerastreamer.camerastreamer       import CameraStreamer
from src.utils.cameraspoofer.cameraspooferprocess  import CameraSpooferProcess
from src.utils.remotecontrol.remotecontrolreceiver import RemoteControlReceiver
from src.utils.autonomous.perceptionprocess        import PerceptionProcess
from src.utils.autonomous.signDetectionprocess     import SignDetectionProcess
from src.utils.autonomous.logicprocess             import LogicProcess

# =============================== CONFIG =================================================
enableStream        =  True
enableStreamPerception = False
enableCameraSpoof   =  False
enableRc            =  False
enablePerception    =  True
#================================ PIPES ==================================================


# gpsBrR, gpsBrS = Pipe(duplex = False)           # gps     ->  brain
#================================ PROCESSES ==============================================
allProcesses = list()

# =============================== HARDWARE PROCC =========================================
# ------------------- camera + streamer ----------------------
if enableStream:
    camRPh, camSPh = Pipe(duplex = False) # -> photo transfering from camera to perception
    camStRP, camStSP = Pipe(duplex = False) # -> perception photo for streaming
    camStR, camStS = Pipe(duplex = False) # -> initial camera frame for streaming
    
    if enablePerception is False:
        camProcess = CameraProcess([],[camStS])
        allProcesses.append(camProcess)
        
        streamProc = CameraStreamer([camStR], [])
        allProcesses.append(streamProc)
    
    else:
        perc2logR, perc2logS = Pipe(duplex = False)
        comR, comS = Pipe(duplex = False)
        signInitR, signInitS = Pipe(duplex = False)
        signR, signS = Pipe(duplex = False)
        
        camProcess = CameraProcess([],[camSPh])
        allProcesses.append(camProcess)
        
        percProc = PerceptionProcess([camRPh, signR], [camStSP, perc2logS, signInitS])
        allProcesses.append(percProc)
        
        logProc = LogicProcess([perc2logR], [comS])
        allProcesses.append(logProc)
        
        signProc = SignDetectionProcess([signInitR], [signS])
        allProcesses.append(signProc)
        
        shProc = SerialHandler([comR], [])
        allProcesses.append(shProc)
        
        streamPerc = CameraStreamer([camStRP], [])
        allProcesses.append(streamPerc)

# =============================== DATA ===================================================
#gps client process
# gpsProc = GpsProcess([], [gpsBrS])
# allProcesses.append(gpsProc)

# ===================================== CONTROL ==========================================
#------------------- remote controller -----------------------
if enableRc:
    rcShR, rcShS   = Pipe(duplex = False)           # rc      ->  serial handler

    # serial handler process
    shProc = SerialHandler([rcShR], [])
    allProcesses.append(shProc)

    rcProc = RemoteControlReceiver([],[rcShS])
    allProcesses.append(rcProc)

print("Starting the processes!",allProcesses)
for proc in allProcesses:
    proc.daemon = True
    proc.start()

blocker = Event()

try:
    blocker.wait()
except KeyboardInterrupt:
    print("\nCatching a Keyboard Interruption exception! Shutdown all processes.\n")
    
    for i in range (0, 500):
        allProcesses[2].reset.value = 1
        time.sleep(0.001)
    
    proc_counter = 0
    for proc in allProcesses:
        proc_counter += 1
        if hasattr(proc,'stop') and callable(getattr(proc,'stop')):
            print("Process with stop",proc)
            if proc_counter == 3:
                proc.reset.value = 1
                time.sleep(2)
            proc.stop()
            proc.join()
        else:
            print("Process without stop",proc)
            proc.terminate()
            proc.join()
