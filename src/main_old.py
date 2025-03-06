import cv2 as cv
import board
import busio
import time
from picamera2 import Picamera2
from libcamera import controls
import ultrasonic
from adafruit_bno055 import BNO055_I2C
import numpy as np
import argparse
import pandas as pd

import config   
import vision
import motors
import sensors
import leds

old_speed = 0

def read_imu_euler(imu,i2c):
    value = imu.euler
    i2c.unlock()
    return value


def main(args):
    global old_speed
    picam = Picamera2()
    conf = picam.create_still_configuration({"size":(1920,1080)})
    picam.configure(conf)
    picam.set_controls({
    "AfMode":controls.AfModeEnum.Continuous,
    "ExposureTime":400,

    })
    picam.start()
    time.sleep(1)
    
    data = config.load('config.json')
    i2c = busio.I2C(board.SCL, board.SDA)
    draw = bool(data['aruco']['draw'])
    fps = data['fps']
    while not i2c.try_lock():
        pass
    try:
        print("Motor --- I2C")
        motor = motors.Motor(i2c,data['motor']['channels'],data['motor']['channel'])
        departure_speed = data['motor']['departure']
        departure = False
        time.sleep(1)
        motor.set_speed(0)
        print("IMU --- I2C")  
        imu = BNO055_I2C(i2c)
        time.sleep(1)
        
        back_ultrasonic = ultrasonic.GroveUltrasonicRanger(data['ultrasonics']['back']['pin'])
        front_ultrasonic = ultrasonic.GroveUltrasonicRanger(data['ultrasonics']['front']['pin'])
        dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
        detector = vision.Detector(dictionary,data['aruco']['ID'],
                                   data['aruco']['size'],data['aruco']['real_size'])
        detector.load_calibration()
        print("LEDS")
        led_front = leds.Led(config.get_led_pins(data, 'led_front'), 
                            data['leds']['led_front']['num_leds'])
        led_back = leds.Led(config.get_led_pins(data, 'led_back'), 
                            data['leds']['led_back']['num_leds'])
        led_bottom = leds.Led(config.get_led_pins(data, 'led_bottom'),
                            data['leds']['led_bottom']['num_leds'])
        #leds.starting_up([led_front, led_back, led_bottom])
        threshold_cst = data['ultrasonics']['fw-threshold']
        #led_bottom.set_color(leds.WHITE)
        last_time = time.time()
        start = True
        detected = False
        started = False
        offset = 0.3
        speed_array = []
        timestamp_array = []
        detected_array = []
        speed=0
        print("FPS",fps)
        # Forward motion, camera tracking #
        if not args.backward:
            print('Forward motion')
            while start:
                threshold = threshold_cst + old_speed * 380
                start = sensors.end_of_line(front_ultrasonic.get_distance(), threshold, motor, led_front, led_back)
                if not start:
                    print('End of the line')
                    break
                timestamp = time.time()
                dt = timestamp-last_time
                if 1/fps < dt:
                    last_time = time.time()
                    print("FPS:",1/dt)
                    # print( time.time() - last_time)
                    # Detection of the end of the line #
                    
                    # IMU #
                    # Vision #
                    picam.capture_file('cam.jpg')
                    frame = cv.imread('cam.jpg')
                    detected,_,translation,delta_x,_,_ = detector.markers_detection(frame, draw)
                    timestamp_array.append(timestamp)
                    if not detected:
                        detected_array.append(0)
                        print("Not detected")
                        if args.forward:
                            
                            if old_speed <0.45:
                                speed = old_speed + 0.1
                            else:
                                speed = 1

                            

                            motor.set_speed(speed)
                            old_speed = speed
                        else:
                            # led_back.set_color(leds.RED)
                            # led_front.set_color(leds.RED)
                            #motor.stop()
                            speed = old_speed
                            speed_array.append(speed)
                            motor.set_speed(speed)
                    else:
                        detected_array.append(1)
                        if not started and delta_x > 0:
                            print('Started')
                            started = True
                        print('Marker detected')
                        # led_front.set_color(leds.GREEN)
                        # led_back.set_color(leds.GREEN)
                        # Motor #
                        #print(translation)
                        euler = read_imu_euler(imu,i2c)
                        if started:
                            speed = motors.speed_controller(delta_x, euler[2], translation[1],old_speed,dt)
                            speed += offset
                            offset = 0      
                        if old_speed == 0:
                            motor.set_speed(np.sign(speed)*0.05)
                            time.sleep(0.1)
                        speed_array.append(speed)
                        motor.set_speed(speed)
                        old_speed = speed
                        print('speed == ',speed)    
                       
                    
                    
                else:
                    continue
        
        # Waiting time at the end of the line #
        time.sleep(3)
        threshold = data['ultrasonics']['bw-threshold']
        start = True
        old_speed = 0
        print('Backward motion')
        # Backward motion, no camera tracking #
        while start:
            start = sensors.end_of_line(back_ultrasonic.get_distance(), threshold, motor, led_front, led_back)
            speed = -0.5
            if old_speed == 0:
                motor.set_speed(np.sign(speed)*0.05)
                time.sleep(0.1)
                motor.set_speed(np.sign(speed)*0.1)
                time.sleep(0.1)

            motor.set_speed(speed)
            old_speed = speed
            if not start:
                print('End of the line')
                motor.stop()
                break
    finally:
        print(len(speed_array),len(timestamp_array),len(detected_array))
        if len(speed_array) > 0 and len(timestamp_array) > 0 and len(detected_array) > 0:
            dict = {'speed':speed_array,'detected':detected_array,'timestamp':timestamp_array}
            df = pd.DataFrame(dict)
            ts = time.time()
            date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(ts))
            path = 'csvs/'+str(date)+'.csv'
            df.to_csv(path)
        i2c.unlock()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-f','--forward',action='store_true',help='Go forward')
    parser.add_argument('-b','--backward',action='store_true',help='Go backward')
    parser.add_argument('-fb',action='store_true',help='Go forward and then backward')
    args = parser.parse_args()
    print('Start')
    main(args)
    print('Stop')
