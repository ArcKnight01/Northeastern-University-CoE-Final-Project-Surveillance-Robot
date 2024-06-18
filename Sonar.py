from Sensor import Sensor
from gpiozero import DistanceSensor
import time

class Sonar(Sensor):
    def __init__(self, verbose=False, enabled=True, echo_pin=0, trig_pin=1):
        super().__init__(verbose=verbose, enabled=enabled)
        self.__distance = 0
        if(self.__enabled):
            self.__sensor = DistanceSensor(echo=echo_pin, trigger=trig_pin)
            self.__sensor.max_distance = 100*100 #cm

    def update(self):
        if(self.__enabled):
            self.__distance = self.__sensor.distance * 100 #cm
            
    def get_distance(self):
        if(self.__enabled):
            self.update()
            if(self.__verbose):
                    print(f"[SONAR SENSOR] Distance: {self.__distance} cm.", end="|")  
        return(self.__distance)
    
    def avoidance_check(self, threshold):
        if(self.__enabled):
            distance = self.get_distance()
            if (distance < threshold):
                if(self.__verbose):
                    print(f"[SONAR SENSOR] Obstacle detected ({self.__distance} cm away)")
                return(True)
            elif (distance  > threshold):
                if(self.__verbose):
                    print(f"[SONAR SENSOR] No obstacle detected ({self.__distance} cm away)")
                return(False)
    

    

        
        
