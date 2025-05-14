import time
import csv
from pathlib import Path
from multiprocessing import Process, Value
from collections import deque
import gpiod
from picamera2.encoders import H264Encoder, Quality
from picamera2.outputs import FfmpegOutput
from datetime import datetime
import numpy as np
from simple_pid import PID
import os
import serial

import config
import state_machine
import leds_ctrl
import ultrasonic_ctrl
import motor_ctrl
import camera_ctrl
import airspeed_sensor_ctrl
import plot_motor_data
import plot_li550_data


def rc_receiver_reading(shared_remote_command, shared_target_speed, shared_calibration_setpoints):
    try:
        with gpiod.Chip('gpiochip0') as chip:
            throttle_line = chip.get_line(config.TROTTLE_PIN)
            button_line = chip.get_line(config.BUTTON_PIN)
            steering_line = chip.get_line(config.STEERING_PIN)

            throttle_line.request(consumer='pwm_reader', type=gpiod.LINE_REQ_EV_BOTH_EDGES)
            button_line.request(consumer='pwm_reader', type=gpiod.LINE_REQ_EV_BOTH_EDGES)
            steering_line.request(consumer='pwm_reader', type=gpiod.LINE_REQ_EV_BOTH_EDGES)

            # Initialize variables
            last_rising_throttle = None
            throttle_timeout = False
            throttle_pulse = 0.0

            last_rising_button = None
            button_timeout = False
            button_pulse = 0.0

            last_rising_steering = None
            steering_timeout = False
            steering_pulse = 0.0
            
            remote_command = 0
            last_remote_command = 0
            calibration_setpoints = 0
            target_speed = 0.0
            initial_button_pulse = 0.0
            button_in_default_position = True
            button_initialized = False

            while True:
                throttle_event = throttle_line.event_wait(sec=1)
                button_event = button_line.event_wait(sec=1)
                steering_event = steering_line.event_wait(sec=1)

                # Reading throttle event
                if throttle_event:
                    ev_throttle = throttle_line.event_read()
                    timestamp_throttle = ev_throttle.sec + ev_throttle.nsec / 1e9
                    if ev_throttle.type == gpiod.LineEvent.RISING_EDGE:
                        last_rising_throttle = timestamp_throttle
                    elif ev_throttle.type == gpiod.LineEvent.FALLING_EDGE and last_rising_throttle is not None:
                        throttle_pulse = (timestamp_throttle - last_rising_throttle) * 1_000_000  # in µs
                        last_rising_throttle = None
                    throttle_timeout = False
                else:
                    throttle_pulse = 0.0
                    throttle_timeout = True

                # Reading button event
                if button_event:
                    ev_button = button_line.event_read()
                    timestamp_button = ev_button.sec + ev_button.nsec / 1e9
                    if ev_button.type == gpiod.LineEvent.RISING_EDGE:
                        last_rising_button = timestamp_button
                    elif ev_button.type == gpiod.LineEvent.FALLING_EDGE and last_rising_button is not None:
                        button_pulse = (timestamp_button - last_rising_button) * 1_000_000  # in µs
                        last_rising_button = None
                    button_timeout = False
                else:
                    button_pulse = 0.0
                    button_timeout = True
                
                # Reading steering event
                if steering_event:
                    ev_steering = steering_line.event_read()
                    timestamp_steering = ev_steering.sec + ev_steering.nsec / 1e9
                    if ev_steering.type == gpiod.LineEvent.RISING_EDGE:
                        last_rising_steering = timestamp_steering
                    elif ev_steering.type == gpiod.LineEvent.FALLING_EDGE and last_rising_steering is not None:
                        steering_pulse = (timestamp_steering - last_rising_steering) * 1_000_000  # in µs
                        last_rising_steering = None
                    steering_timeout = False
                else:
                    steering_pulse = 0.0
                    steering_timeout = True
                
                # Logic to determine remote command, target speed, and calibration setpoints
                if throttle_timeout or (throttle_pulse == 0.0) or button_timeout or (button_pulse == 0.0) or steering_timeout or (steering_pulse == 0.0):
                    if last_remote_command == 3:
                        remote_command = 3  # stay in "GO_TRACKING" if deconnection and was in tracking mode
                        target_speed = 0.0
                        calibration_setpoints = 0  # no setpoint
                    else:
                        remote_command = 0 # change to "GO_STOP" if deconnection and was in manual mode
                        target_speed = 0.0
                        calibration_setpoints = 0  # no setpoint
                else:                                                      
                    # If this is the first button pulse read, use it as the reference
                    if not button_initialized:
                        initial_button_pulse = button_pulse
                        button_initialized = True
                        
                    # Determine if the button is in the same position as initial (GO_STOP) or toggled (GO_TRACKING)
                    if (abs(button_pulse - initial_button_pulse) < config.BUTTON_TOGGLE_THRESHOLD) and button_initialized:
                        button_in_default_position = True # button is in the same position as initial
                    else:
                        button_in_default_position = False # button is in the opposite position as initial

                    # Logic for remote command and target speed
                    if remote_command == 3:
                        # Tracking mode
                        if (button_in_default_position) or abs(throttle_pulse - config.PWM_DEFAULT_PULSE_WIDTH) > config.STAY_TRACKING_THRESHOLD:
                            # if the button or the trottle is touched, stop tracking
                            remote_command = 0 
                            target_speed = 0.0
                            initial_button_pulse = button_pulse  # so that it require a button toggle to resume tracking
                        else:
                            # if the button is not touched, keep tracking
                            remote_command = 3
                            target_speed = 0.0
                    else:
                        # Manual mode
                        if not button_in_default_position:
                            # if the button is toggled, start tracking
                            remote_command = 3
                            target_speed = 0.0
                        else:
                            if abs(throttle_pulse - config.PWM_DEFAULT_PULSE_WIDTH) < config.GO_STOP_THRESHOLD:
                                remote_command = 0  # corresponding to "GO_STOP"
                                target_speed = 0.0
                            elif throttle_pulse < (config.PWM_DEFAULT_PULSE_WIDTH - config.GO_STOP_THRESHOLD):
                                remote_command = 1  # corresponding to "GO_BACKWARD"
                                target_speed = throttle_pulse
                            elif throttle_pulse > (config.PWM_DEFAULT_PULSE_WIDTH + config.GO_STOP_THRESHOLD):
                                remote_command = 2  # corresponding to "GO_FORWARD"
                                target_speed = throttle_pulse
                    
                    # Logic for calibration setpoints
                    if abs(steering_pulse - config.PWM_MIN_PULSE_WIDTH) < config.CALIB_SETPOINTS_THRESHOLD:
                        calibration_setpoints = 1 # setpoints for zipline start
                    elif abs(steering_pulse - config.PWM_MAX_PULSE_WIDTH) < config.CALIB_SETPOINTS_THRESHOLD:
                        calibration_setpoints = 2 # setpoints for zipline length
                    else:
                        calibration_setpoints = 0 # no setpoint

                last_remote_command = remote_command

                # Update shared values between processes
                with shared_remote_command.get_lock(), shared_target_speed.get_lock(), shared_calibration_setpoints.get_lock():
                    shared_remote_command.value = remote_command
                    shared_target_speed.value = target_speed
                    shared_calibration_setpoints.value = calibration_setpoints

                time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nRC remote process stopped.")
    finally:
        throttle_line.release()
        button_line.release()
        steering_line.release()


def camera_process(save_path, shared_offset, shared_detect_flag):
    camera = camera_ctrl.camera_init()
    mtx, dist = camera_ctrl.load_calibration()
    frame_counter = 1

    # Data recording settings    
    timestamp_folder = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    save_path = Path(save_path) / f"frames_{timestamp_folder}"
    save_path.mkdir(parents=True, exist_ok=True)
    try:
        while True:
            time_start_while = time.time()

            # Only compute if tracking is on
            if shared_detect_flag.value == 1:
                offset = camera_ctrl.detect_aruco_pose(camera, mtx, dist, save_path, frame_counter)
                frame_counter += 1
                with shared_offset.get_lock():
                    shared_offset.value = offset if offset is not None else float('nan')
            else:
                time.sleep(0.01)
            
            time_end_while = time.time()
            # print("Camera process:", time_end_while - time_start_while, "s") # 0.01s without ArUco, 0.03s with ArUco
            if time_end_while - time_start_while < config.DT_VISION:
                time.sleep(config.DT_VISION - (time_end_while - time_start_while))
            else:
                print(f"Camera process: Execution time exceeded {config.DT_VISION} seconds.")
    except KeyboardInterrupt:
        print("\nCamera process stopped.")
    finally:
        camera.stop()


def main(save_path, shared_remote_command, shared_target_speed, shared_calibration_setpoints, shared_offset, shared_detect_flag):
    # Initialize peripherals
    leds = leds_ctrl.leds_init()
    odrv = motor_ctrl.motor_init()
    #print(odrv)
    # ser = serial.Serial(config.SERIAL_PORT_LI550, config.BAUD_RATE_LI550, timeout=1)
    # print("Waiting for LI550 to be ready...")
    # time.sleep(config.INIT_TIME_LI550) # wait for the LI550 to be ready

    # Variable initialization
    time_start_while = 0
    time_end_while = 0
    time_start_abs = time.time()
    leds_off_before = False
    state = config.STATE["STOP"]
    last_state = state
    tracking_error = None
    last_tracking_error = None
    cnt_moving_blindly = 0
    obstacle_forward = False
    obstacle_backward = False
    decelerating_to_full_stop = False
    calibration_mode = True
    first_time_calibration_mode = True
    first_time_normal_mode = True
    zipline_length = 0
    zipline_start = 0
    zipline_end = 0
    zipline_start_set = False
    zipline_end_set = False

    # Data recording setings 
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_path = Path(save_path) / f"data_{timestamp}.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    csv_file = open(csv_path, 'w', newline='')
    writer = csv.DictWriter(csv_file, fieldnames=config.CSV_COLUMNS)
    writer.writeheader()

    try:
        try:
            while True:
                # Get remote control commands
                with shared_remote_command.get_lock(), shared_target_speed.get_lock(), shared_calibration_setpoints.get_lock():
                    remote_command = shared_remote_command.value                # 0: no command, 1: go backward, 2: go forward, 3: go tracking
                    target_speed = shared_target_speed.value                    # in µs
                    calibration_setpoints = shared_calibration_setpoints.value  # 0: no setpoint, 1: zipline start, 2: zipline length

                    # Mapping target_speed in µs to target_speed_m_s in m/s
                    if remote_command == 1:
                        if calibration_mode:
                            target_speed_m_s = config.MAX_MANUAL_SPEED_CALIB - np.interp(target_speed, [config.PWM_MIN_PULSE_WIDTH, config.PWM_DEFAULT_PULSE_WIDTH], [0.0, config.MAX_MANUAL_SPEED_CALIB])
                        else:
                            target_speed_m_s = config.MAX_MANUAL_SPEED - np.interp(target_speed, [config.PWM_MIN_PULSE_WIDTH, config.PWM_DEFAULT_PULSE_WIDTH], [0.0, config.MAX_MANUAL_SPEED])
                    elif remote_command == 2:
                        if calibration_mode:
                            target_speed_m_s = np.interp(target_speed, [config.PWM_DEFAULT_PULSE_WIDTH, config.PWM_MAX_PULSE_WIDTH], [0.0, config.MAX_MANUAL_SPEED_CALIB])
                        else:
                            target_speed_m_s = np.interp(target_speed, [config.PWM_DEFAULT_PULSE_WIDTH, config.PWM_MAX_PULSE_WIDTH], [0.0, config.MAX_MANUAL_SPEED])
                    else:
                        target_speed_m_s = 0.0
                
                # Get motor data                    
                angular_position, angular_velocity, torque, linear_position, linear_velocity, voltage, current = motor_ctrl.get_data(odrv) # in turns, turns/s, Nm, m, m/s
                    
                # Calibration logic
                if calibration_mode:
                    if first_time_calibration_mode:
                        odrv.axis0.pos_estimate = motor_ctrl.compute_angular_position(-config.INITAL_MOTOR_POS_CALIB)
                        x_ref = config.INITAL_MOTOR_POS_CALIB
                        last_x_ref = config.INITAL_MOTOR_POS_CALIB
                        first_time_calibration_mode = False

                    if calibration_setpoints == 2:
                        # Set the zipline length
                        zipline_end = linear_position
                        zipline_end_set = True
                        
                    if (calibration_setpoints == 1) and zipline_end_set:
                        # Set the zipline start
                        zipline_start = linear_position
                        zipline_start_set = True
                        zipline_length = zipline_end - zipline_start
                        calibration_mode = False
                        print(f"Calibration done: zipline length = {zipline_length:.2f} m")                        
                else:
                    if first_time_normal_mode:
                        odrv.axis0.pos_estimate = motor_ctrl.compute_angular_position(0) # Start at 0 m 
                        x_ref = 0
                        last_x_ref = 0
                        first_time_normal_mode = False
                    
                    
                if (odrv.axis0.active_errors != 0) or (odrv.axis0.disarm_reason != 0) or (zipline_length < 0) or (zipline_start < 0):
                    print(f"ODrive error: {odrv.axis0.active_errors}  Disarm reason: {odrv.axis0.disarm_reason}", "Zipline start:", zipline_start, "Zipline length:", zipline_length)
                    
                    leds_off_before = leds_ctrl.leds_error_warning(leds, leds_off_before)
                    motor_ctrl.set_position(odrv, - odrv.axis0.pos_estimate) # stay in the same position
                    time.sleep(config.DT)
                else:
                    time_start_while = time.time()
                
                    # Read data from the LI-5500 sensor
                    # li550_data = airspeed_sensor_ctrl.log_li550_data(ser)
                    li550_data = {}

                    # Get distance sensor readings
                    obstacle_forward, obstacle_backward = ultrasonic_ctrl.is_there_an_obstacle()

                    # Update state
                    state, reached_end, reached_start = state_machine.update(last_state, remote_command, obstacle_forward, obstacle_backward, x_ref, angular_velocity, decelerating_to_full_stop, calibration_mode, zipline_length)
                    last_state = state

                    if state == config.STATE["TRACKING"]:
                        # Start camera recording and get ArUco marker position
                        with shared_detect_flag.get_lock():
                            shared_detect_flag.value = 1
                        with shared_offset.get_lock():
                            tracking_error = shared_offset.value

                        tracking_error = tracking_error if not np.isnan(tracking_error) else None # in m

                        if tracking_error is not None:
                            # ArUco marker detected: compute target speed using PID controller
                            x_ref = last_x_ref + tracking_error
                            cnt_moving_blindly = 0
                        else:
                            # ArUco marker not detected: keep the last target speed for a while
                            if (last_tracking_error is not None) and cnt_moving_blindly < config.MAX_CNT_MOVING_BLINDLY:
                                tracking_error = last_tracking_error
                                x_ref = last_x_ref + tracking_error
                                cnt_moving_blindly += 1
                            else:
                                # No ArUco marker detected and no previous offset: stop the motor
                                x_ref = linear_position
                        
                        last_tracking_error = tracking_error
                    else:
                        with shared_detect_flag.get_lock():
                            shared_detect_flag.value = 1    # saves frames from camera (should be 0)     !!FIXME!!
                        tracking_error = None
                        last_tracking_error = None

                        if state == config.STATE["STOP"]:
                            decelerating_to_full_stop = True

                            if linear_velocity < config.STOP_SPEED_THRESHOLD:
                                decelerating_to_full_stop = False

                            if reached_end:
                                x_ref = config.ZIPLINE_LENGTH_CALIB if calibration_mode else zipline_length
                            elif reached_start:
                                x_ref = config.ZIPLINE_START_CALIB if calibration_mode else 0
                            else:
                                x_ref = linear_position # Stay in the same position, thus max deceleration

                        elif state == config.STATE["FORWARD"]:
                            x_ref = linear_position + target_speed_m_s * config.DT * config.BANG_BANG_GAIN # Move forward

                        elif state == config.STATE["BACKWARD"]:
                            x_ref = linear_position - target_speed_m_s * config.DT * config.BANG_BANG_GAIN # Move backward

                        else:
                            x_ref = linear_position # Stay in the same position, thus max deceleration

                    motor_ctrl.set_position(odrv, motor_ctrl.compute_angular_position(x_ref))
                    last_x_ref = x_ref

                    # Display current state with LEDs
                    leds_off_before = leds_ctrl.leds_set_color(leds, state, obstacle_forward, obstacle_backward, tracking_error, leds_off_before, calibration_mode)

                    if not calibration_mode:
                        # Save data to CSV
                        motor_data = motor_ctrl.log_motor_data(time.time() - time_start_abs, angular_position, angular_velocity, torque, linear_position, linear_velocity, tracking_error, voltage, current, x_ref)
                        row = {**motor_data, **li550_data}
                        writer.writerow(row)
                        csv_file.flush()

                        # Print current state
                        offset_str = "N/A" if tracking_error is None else f"{tracking_error:.2f} m"
                        log_message = (f"Command: {config.COMMAND_LOOKUP.get(remote_command, 'UNKNOWN')}  |  "
                                    f"Backward obst.: {obstacle_backward}  |  Forward obst.: {obstacle_forward}  |  "
                                    f"State: {config.STATE_LOOKUP.get(state, 'UNKNOWN')}  |   "
                                    f"Position: {linear_position:.2f} m  |  "
                                    f"Velocity: {linear_velocity:.2f} m/s\n"
                                    f"ArUco position: {offset_str} m  |  Target velocity: {target_speed_m_s:.2f} m/s")
                        print(log_message)

                    # Sleep to respect the desired loop time
                    time_end_while = time.time()
                    # print("Main process:", time_end_while - time_start_while, "s") # 0.05s usually
                    if time_end_while - time_start_while < config.DT:
                        time.sleep(config.DT - (time_end_while - time_start_while))
                    else:
                        print(f"Main process: Execution time exceeded {config.DT} seconds.")

        except KeyboardInterrupt:
            print("\nMain process stopped.")
            motor_ctrl.set_position(odrv, - odrv.axis0.pos_estimate) # stay in the same position
            leds_off_before = leds_ctrl.leds_set_color(leds, config.STATE["STOP"], obstacle_forward, obstacle_backward, tracking_error, leds_off_before)

    finally:
        csv_file.close()
        print(f"\nRun complete. Data saved to {csv_path}")
        try:
            print("Plotting data...")
            plot_motor_data.plot_data(csv_path)
            # plot_li550_data.plot_data(csv_path)
            # plot_li550_data.create_video_from_data(csv_path)
        except Exception as e:
            print(f"Plotting failed: {e}")


if __name__ == "__main__":
    # Create a folder to save the data
    timestamp_folder = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    save_path = Path("data") / f"run_{timestamp_folder}"
    os.makedirs(save_path, exist_ok=True)

    # Shared variables for inter-process communication
    shared_remote_command = Value('i', 0)  # remote_command
    shared_target_speed = Value('d', 0.0)  # target_speed
    shared_calibration_setpoints = Value('d', 0.0)  # calibration_setpoints

    shared_offset = Value('d', float('nan'))  # ArUco detection result
    shared_detect_flag = Value('i', 0)        # flag to compute detection

    p1 = Process(target=rc_receiver_reading, args=(shared_remote_command, shared_target_speed, shared_calibration_setpoints))
    p2 = Process(target=camera_process, args=(save_path, shared_offset, shared_detect_flag))
    p3 = Process(target=main, args=(save_path, shared_remote_command, shared_target_speed, shared_calibration_setpoints, shared_offset, shared_detect_flag))

    p1.start()
    p2.start()
    p3.start()

    p1.join()
    p2.join()
    p3.join()