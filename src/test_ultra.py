import time
import gpiod

import config

# Constants
CHIP_NAME = 'gpiochip0'

def usleep(microseconds):
    """
    Sleep for a given number of microseconds.
    :param microseconds: Time to sleep in microseconds.
    """
    time.sleep(microseconds / 1_000_000.0)

def get_distance(sensor_pin):
    """
    Get the distance from the ultrasonic sensor.
    :param sensor_pin: GPIO pin number.
    :return: Distance in meters.
    """
    with gpiod.Chip(CHIP_NAME) as chip:
        line = chip.get_line(sensor_pin)

        # Set up the line for output (trigger)
        line.request(consumer='ultrasonic', type=gpiod.LINE_REQ_DIR_OUT)
        line.set_value(0)
        usleep(2)
        line.set_value(1)
        usleep(10)
        line.set_value(0)
        line.release()

        # Set up the line for input (echo)
        line.request(consumer='ultrasonic', type=gpiod.LINE_REQ_DIR_IN)

        t0 = time.time()
        count = 0
        while count < config.TIMEOUT1:
            if line.get_value() == 1:
                break
            count += 1
        if count >= config.TIMEOUT1:
            line.release()
            return config.MAX_DIST

        t1 = time.time()
        count = 0
        while count < config.TIMEOUT2:
            if line.get_value() == 0:
                break
            count += 1
        if count >= config.TIMEOUT2:
            line.release()
            return config.MAX_DIST

        t2 = time.time()
        dt = int((t1 - t0) * 1_000_000)
        if dt > 530:
            line.release()
            return config.MAX_DIST

        distance = ((t2 - t1) * 1_000_000 / 29 / 2) / 100  # in meters
        line.release()
        return distance

def main():
    while True:
        front_value = get_distance(config.PIN_FRONT)
        back_value = get_distance(config.PIN_BACK)
        print('Back: {:.2f} m    |    Front: {:.2f} m'.format(back_value, front_value))
        time.sleep(0.1)

if __name__ == "__main__":
    main()