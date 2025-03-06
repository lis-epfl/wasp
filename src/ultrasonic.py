import time
import gpiozero
from gpiozero import Device
from gpiozero.pins.lgpio import LGPIOFactory
import numpy as np

import main


def usleep(microseconds):
    time.sleep(microseconds / 1000000.0)


def get_distance(PIN):
    """
    Get the distance from the ultrasonic sensor
    :param PIN: GPIO pin number
    :return: Distance in meters
    (inspired from manufacturer's code: https://wiki.seeedstudio.com/Grove-Ultrasonic_Ranger/)
    """

    trig = gpiozero.DigitalOutputDevice(PIN)
    trig.off()
    usleep(2)
    trig.on()
    usleep(10)
    trig.off()
    
    trig.close()
    echo = gpiozero.DigitalInputDevice(PIN)

    t0 = time.time()
    count = 0
    while count < TIMEOUT1:
        if echo.value:
            break
        count += 1
    if count >= TIMEOUT1:
        echo.close()
        return MAX_DIST
    
    t1 = time.time()
    count = 0
    while count < TIMEOUT2:
        if not echo.value:
            break
        count += 1
    if count >= TIMEOUT2:
        echo.close()
        return MAX_DIST
    
    t2 = time.time()
    dt = int((t1 - t0) * 1_000_000)
    if dt > 530:
        echo.close()
        return MAX_DIST
    
    distance = ((t2 - t1) * 1_000_000 / 29 / 2) / 100  # in meters
    echo.close()
    return distance

def is_there_obstacle


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
    
    
        
