
# picam2 = Picamera2()
# camera_config = picam2.create_preview_configuration()
# picam2.configure(camera_config)
# picam2.start_preview(Preview.QTGL)

# while(True):
#     picam2.start()
#picam2.capture_file("test.jpg")

from picamera2 import Picamera2, Preview
import time

import config_old   


def main():
     picam = Picamera2()
    conf = picam.create_still_configuration({"size":(1920,1080)})

    picam.configure(conf)
    picam.set_controls({
    "AfMode":controls.AfModeEnum.Continuous,
    "ExposureTime":400,

    })
    picam.start()
    time.sleep(1)
    
    data = config.load('config_old.json')


    draw = bool(data['aruco']['draw'])

    fps = data['fps']

    calibration


def main(args):

   
    while not i2c.try_lock():
        pass
    try:


        dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
        detector = vision.Detector(dictionary,data['aruco']['ID'],
                                   data['aruco']['size'],data['aruco']['real_size'])
        detector.load_calibration()

       
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




if __name__ == "__main__":
    main()

