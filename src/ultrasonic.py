import time
import gpiozero
import numpy as np

usleep = lambda x: time.sleep(x / 1000000.0)

_TIMEOUT1 = 1000
_TIMEOUT2 = 10000

_MAX_DIST = 600


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
        while count < _TIMEOUT1:
            if echo.value:
                break
            count += 1
        if count >= _TIMEOUT1:
            echo.close()
            return None
        
        t1 = time.time()
        count = 0
        while count < _TIMEOUT2:
            if not echo.value:
                break
            count += 1
        if count >= _TIMEOUT2:
            echo.close()
            del echo
            return None
        
        t2 = time.time()
        dt = int((t1 - t0) * 1000000)
        if dt > 530:
            echo.close()
            del echo
            return None
        
        distance = ((t2 - t1) * 1000000 / 29 / 2)    # cm
        echo.close()
        del echo
        return distance
    
    def get_distance(self):
        dist = self._get_distance()
        if dist:
            return dist

        return _MAX_DIST
            
    
    
        
