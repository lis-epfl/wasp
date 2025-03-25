import time
import gpiozero
from gpiozero import Device
from gpiozero.pins.lgpio import LGPIOFactory
import numpy as np

import config

def ultrasonic_init():
    """
    Initialize the ultrasonic sensor
    """
    Device.pin_factory = LGPIOFactory()  # force it to use pigpio to make it work in the venv

def usleep(microseconds):
    """
    Sleep for a given number of microseconds
    :param microseconds: Time to sleep in microseconds
    """
    time.sleep(microseconds / 1000000.0)


def get_distance(sensor_pin):
    """
    Get the distance from the ultrasonic sensor
    :param PIN: GPIO pin number
    :return: Distance in meters
    (inspired from manufacturer's code: https://wiki.seeedstudio.com/Grove-Ultrasonic_Ranger/)
    """
    trig = gpiozero.DigitalOutputDevice(sensor_pin)
    trig.off()
    usleep(2)
    trig.on()
    usleep(10)
    trig.off()
    
    trig.close()
    echo = gpiozero.DigitalInputDevice(sensor_pin)

    t0 = time.time()
    count = 0
    while count < config.TIMEOUT1:
        if echo.value:
            break
        count += 1
    if count >= config.TIMEOUT1:
        echo.close()
        return config.MAX_DIST
    
    t1 = time.time()
    count = 0
    while count < config.TIMEOUT2:
        if not echo.value:
            break
        count += 1
    if count >= config.TIMEOUT2:
        echo.close()
        return config.MAX_DIST
    
    t2 = time.time()
    dt = int((t1 - t0) * 1_000_000)
    if dt > 530:
        echo.close()
        return config.MAX_DIST
    
    distance = ((t2 - t1) * 1_000_000 / 29 / 2) / 100  # in meters
    echo.close()
    return distance