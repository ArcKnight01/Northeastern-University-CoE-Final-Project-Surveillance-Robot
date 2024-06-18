from Servo_Motors import ServoMotor
import time
class CameraMount:
    def __init__(self, servo_pin_1, servo_pin_2, verbose:bool=False, enabled:bool=True):
        self.__verbose = verbose
        self.__top_servo = ServoMotor(pin=servo_pin_1, initial_degree=0)
        self.__bottom_servo = ServoMotor(pin=servo_pin_2, initial_degree=0)
        self.__counter = 0
        
    def moveToSphericalCoordinate(self, top_servo_deg, bottom_servo_deg):
        self.__top_servo.rotateToDegree(top_servo_deg)
        self.__bottom_servo.rotateToDegree(bottom_servo_deg)
        time.sleep(0.001)

    def getSphericalCoordinates(self):
        if(self.__verbose == True):
            print(f"[CAM]T{self.__top_servo.get_degree()},B{self.__bottom_servo.get_degree()}")

        return((self.__top_servo.get_degree(), self.__bottom_servo.get_degree()))
    
    def revolve(self):
        currentTopDeg, currentBottomDeg = self.getSphericalCoordinates()
        
        if(self.__counter == 0):
            self.moveToSphericalCoordinate(currentTopDeg, 0)
        elif(self.__counter == 1):
            self.moveToSphericalCoordinate(currentTopDeg, 90)
        elif(self.__counter == 2):
            self.moveToSphericalCoordinate(currentTopDeg, 180)
        elif(self.__counter == 3):
            self.moveToSphericalCoordinate(currentTopDeg, 90)
        elif(self.__counter == 4):
            self.moveToSphericalCoordinate(currentTopDeg, 0)
        elif(self.__counter == 5):
            self.moveToSphericalCoordinate(currentTopDeg, -90)
        elif(self.__counter == 6):
            self.moveToSphericalCoordinate(currentTopDeg, -180)
        elif(self.__counter == 7):
            self.moveToSphericalCoordinate(currentTopDeg, 90)
        elif(self.__counter == 8):
            self.moveToSphericalCoordinate(currentTopDeg, 0)
        time.sleep(1)
        self.__counter+=1
        if (self.__counter >= 8):
            self.__counter = 0
        

    

if __name__ == '__main__':
    cameraMount = CameraMount(10,9)
    cameraMount.moveToSphericalCoordinate(0,0)
    while(True):
        print(f"Top,Bottom={cameraMount.getSphericalCoordinates()}")
        cameraMount.revolve()
        time.sleep(0.01)