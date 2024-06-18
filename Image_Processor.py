import sys
import pathlib
import datetime

import time
import cv2
import numpy as np
import os
from gpiozero import TonalBuzzer
from gpiozero.tones import Tone
try:
    print(f"{os.uname()}")
    import picamera
except:
    pass

from Camera_Util import detect_apriltags
from Camera_Util import detect_buoys
class ImageProcessor():
    def __init__(self, log_dir:str='./', verbose:bool=False, enabled:bool=True):
        self.__verbose = verbose
        self.__enabled = enabled
        
        self.__camera = picamera.PiCamera()
        self.__camera.resolution = (640, 480)
        self.__camera.framerate = 24
        time.sleep(0.1) #camera warm up time
        self.__image = np.empty((480*640*3,), dtype=np.uint8)
        
        #create image save directory
        self.__image_dir = pathlib.Path(log_dir,'Frames')
        if(self.__image_dir.exists() == False):
            print(f"[INFO] {self.__image_dir} does not exist, creating directory.")
        self.__image_dir.mkdir(parents=True, exist_ok=True)
        self.__buzzer = TonalBuzzer(11)    
    # ------------------------------------------------------------------------ #
    # Run an iteration of the image processor. 
    # The sim version needs the robot_state (a dictionary of values from the ADCS System) ot generate simulated imagery (Deprecated)
    # the PICAM does not need any robot_state input
    # ------------------------------------------------------------------------ #
    reds = []
    def run(self):
        if(self.__enabled):
            try:
                self.__camera.start_preview()
                self.__camera.capture(self.__image, 'bgr')
            except:
                # restart the camera
                # self.__camera = picamera.PiCamera()
                self.__camera.resolution = (640, 480)
                self.__camera.framerate = 24
                time.sleep(0.05) # camera warmup time
                
            image = self.__image.reshape((480, 640, 3))
            reds,_,_,_= detect_buoys(image)
            print(reds)
            if len(reds) != 0: 
                self.__buzzer.play(tone=Tone("A4"))
                time.sleep(1)
                self.__buzzer.stop()
                for red in reds:
                    print(f"RED DETECTED at {red}")
                    

            #detect APRIL TAGS
            # detected, image, tagFamilies, tagIds, centers, angles, corners = detect_apriltags(image)
            # if(detected == True):
            #     if(self.__verbose==True):
            #         print(f"TAG(s) DETECTED:")
            #     for tagId in tagIds:
            #         if(self.__verbose==True):
            #             print(tagId)
            # else:
                
            #     print(f"NO TAG(s) DETECTED!")

            
            #detect SPHERES
            # detect_spheres(image)

            # log the image
            fn = self.__image_dir / f"frame_{int(datetime.datetime.utcnow().timestamp())}.jpg"
            if (self.__verbose ==True):
                print(f"Took image {fn}.")
            cv2.imwrite(str(fn), image)

if __name__ == '__main__':
    from CameraMount import CameraMount
    cameraMount = CameraMount(10,9)
    imageprocessor = ImageProcessor(log_dir='./')
    while(True):
        cameraMount.revolve()
        imageprocessor.run()
