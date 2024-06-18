from Sonar import Sonar
from Sensor import Sensor
class DualSonarSystem(Sensor):
    def __init__(self, verbose: bool = False, enabled: bool = True, left_sonar:Sonar=None, right_sonar:Sonar=None):
        super().__init__(verbose=verbose, enabled=enabled)
        if(self.__enabled):
            assert (left_sonar and right_sonar), ValueError
            self.__sonar_left = left_sonar
            self.__sonar_right = right_sonar
        pass
    
    def get_distances(self):
        if(self.__enabled):
            left_distance = self.__sonar_left.get_distance()
            right_distance = self.__sonar_right.get_distance()
            if(self.__verbose):
                print(f"[DISTANCE SENSOR] Distance (CM): {left_distance} (LEFT), {right_distance} (RIGHT).", end="|")
            return(left_distance, right_distance)
        else:
            return(-1,-1)
    