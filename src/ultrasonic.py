import time
import gpiozero
from gpiozero import Device
from gpiozero.pins.lgpio import LGPIOFactory
import numpy as np

#--------------------------------------------------------CONSTANTS--------------------------------------------------------
TIMEOUT1 = 1000
TIMEOUT2 = 10000

MAX_DIST = 6 # in meters
PIN_FRONT = 5
PIN_BACK = 16
BW_THRESHOLD = 60
FW_THRESHOLD = 100
#-------------------------------------------------------------------------------------------------------------------------

#---------------------CODE FROM MANUFACTURER (https://wiki.seeedstudio.com/Grove-Ultrasonic_Ranger/)----------------------
usleep = lambda x: time.sleep(x / 1000000.0)

class GroveUltrasonicRanger:
    def __init__(self,pin):
        self.pin = pin
        
    def _get_distance(self):
        trig = gpiozero.DigitalOutputDevice(self.pin)
        trig.off()
        usleep(2)
        trig.on()
        usleep(10)
        trig.off()

        trig.close()
        echo = gpiozero.DigitalInputDevice(self.pin)

        t0 = time.time()
        count = 0
        while count < TIMEOUT1:
            if echo.value:
                break
            count += 1
        if count >= TIMEOUT1:
            echo.close()
            return None
        
        t1 = time.time()
        count = 0
        while count < TIMEOUT2:
            if not echo.value:
                break
            count += 1
        if count >= TIMEOUT2:
            echo.close()
            del echo
            return None
        
        t2 = time.time()
        dt = int((t1 - t0) * 1000000)
        if dt > 530:
            echo.close()
            del echo
            return None
        
        distance = ((t2 - t1) * 1000000 / 29 / 2)/100  # in meters
        echo.close()
        del echo
        return distance
    
    def get_distance(self):
        dist = self._get_distance()
        if dist:
            return dist
        return MAX_DIST
#-------------------------------------------------------------------------------------------------------------------------


def main():
    Device.pin_factory = LGPIOFactory() # force it to use pigpio to make it work in the venv

    back_ultrasonic = GroveUltrasonicRanger(PIN_BACK)
    front_ultrasonic = GroveUltrasonicRanger(PIN_FRONT)

    print('Measured distance:')
    while True:
        print('Back: {:.2f} m    |    Front: {:.2f} m'
        .format(back_ultrasonic.get_distance(), front_ultrasonic.get_distance()))
        time.sleep(0.5)

if __name__ == '__main__':
    main()
    
    
        
