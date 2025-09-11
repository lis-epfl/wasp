import time
import csv
from pathlib import Path
from multiprocessing import Process, Value
import gpiod
from picamera2.encoders import H264Encoder, Quality
from picamera2.outputs import FfmpegOutput
from datetime import datetime, date
import numpy as np
import os
import serial
import random

import config
import state_machine
import leds_ctrl
import motor_ctrl
import camera_ctrl
import airspeed_sensor_ctrl
import plot_motor_data
import plot_li550_data
import calibration_file_handling


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
                        remote_command = 3          # stay in "GO_TRACKING" if deconnection and was in tracking mode
                        target_speed = 0.0
                        calibration_setpoints = 0   # no setpoint
                    else:
                        remote_command = 0          # change to "GO_STOP" if deconnection and was in manual mode
                        target_speed = 0.0
                        calibration_setpoints = 0   # no setpoint
                else:                                                      
                    # If this is the first button pulse read, use it as the reference
                    if not button_initialized:
                        initial_button_pulse = button_pulse
                        button_initialized = True
                        
                    # Determine if the button is in the same position as initial (GO_STOP) or toggled (GO_TRACKING)
                    if (abs(button_pulse - initial_button_pulse) < config.BUTTON_TOGGLE_THRESHOLD) and button_initialized:
                        button_in_default_position = True   # button is in the same position as initial
                    else:
                        button_in_default_position = False  # button is in the opposite position as initial

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


def camera_process(save_path, time_start_ref, shared_x_aruco, shared_y_aruco, shared_z_aruco, shared_roll_aruco, shared_pitch_aruco, shared_yaw_aruco, shared_time_frame_captured, shared_detect_flag):
    camera = camera_ctrl.camera_init()
    mtx, dist = camera_ctrl.load_calibration()
    frame_counter = 1
    time_frame_captured = 0
    ArUco_pose = {
        'x ArUco [m]': None,
        'y ArUco [m]': None,
        'z ArUco [m]': None,
        'roll ArUco [deg]': None,
        'pitch ArUco [deg]': None,
        'yaw ArUco [deg]': None,
    }

    # Data recording settings    
    timestamp_folder = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    
    save_path = Path(save_path) / f"frames_{timestamp_folder}"
    save_path.mkdir(parents=True, exist_ok=True)
    try:
        while True:
            time_start_while = time.time()

            # Only compute if tracking is on
            if shared_detect_flag.value == 1:

                ArUco_pose, time_frame_captured = camera_ctrl.detect_aruco_pose(camera, mtx, dist, save_path, frame_counter, time_start_ref)
                frame_counter += 1

                with shared_x_aruco.get_lock(), shared_y_aruco.get_lock(), shared_z_aruco.get_lock(), shared_roll_aruco.get_lock(), shared_pitch_aruco.get_lock(), shared_yaw_aruco.get_lock(), shared_time_frame_captured.get_lock():
                    shared_x_aruco.value = - ArUco_pose['x ArUco [m]'] if ArUco_pose['x ArUco [m]'] is not None else float('nan')
                    shared_y_aruco.value = ArUco_pose['y ArUco [m]'] if ArUco_pose['y ArUco [m]'] is not None else float('nan')
                    shared_z_aruco.value = ArUco_pose['z ArUco [m]'] if ArUco_pose['z ArUco [m]'] is not None else float('nan')
                    shared_roll_aruco.value = ArUco_pose['roll ArUco [deg]'] if ArUco_pose['roll ArUco [deg]'] is not None else float('nan')
                    shared_pitch_aruco.value = ArUco_pose['pitch ArUco [deg]'] if ArUco_pose['pitch ArUco [deg]'] is not None else float('nan')
                    shared_yaw_aruco.value = ArUco_pose['yaw ArUco [deg]'] if ArUco_pose['yaw ArUco [deg]'] is not None else float('nan')
                    shared_time_frame_captured.value = time_frame_captured
            else:
                time.sleep(0.01)
            
            time_end_while = time.time()
            # if ArUco_pose['x ArUco [m]'] is not None:
            #     print(f"ArUco Marker Position - X: {ArUco_pose['x ArUco [m]']}, Y: {ArUco_pose['y ArUco [m]']}, Z: {ArUco_pose['z ArUco [m]']}")
            #     print(f"ArUco Marker Orientation - Roll: {ArUco_pose['roll ArUco [deg]']}, Pitch: {ArUco_pose['pitch ArUco [deg]']}, Yaw: {ArUco_pose['yaw ArUco [deg]']}")
            # print("Camera process:", time_end_while - time_start_while, "s") # ~0.12s with full resolution
            if time_end_while - time_start_while < config.DT_VISION:
                time.sleep(config.DT_VISION - (time_end_while - time_start_while))
            else:
                print(f"Camera process: Execution time exceeded {config.DT_VISION} seconds.")
    except KeyboardInterrupt:
        print("\nCamera process stopped.")
    finally:
        camera.stop()


def main(save_path, time_start_ref, shared_remote_command, shared_target_speed, shared_calibration_setpoints, shared_x_aruco, shared_y_aruco, shared_z_aruco, shared_roll_aruco, shared_pitch_aruco, shared_yaw_aruco, shared_time_frame_captured, shared_detect_flag):
    # Initialize variable
    time_start_while = 0
    time_end_while = 0
    leds_off_before = False
    state = config.STATE["STOP"]
    last_state = state
    x_stop = 0
    start_decelerating = True
    decelerating = True

    tracking_error = None
    last_tracking_error = None
    last_target_position = None
    last_time_frame_captured = None
    estimated_velocity = 0.0
    last_estimated_velocity = 0.0
    estimated_position = 0.0
    cnt_moving_blindly = 0

    in_calibration_mode = True
    zipline_end_set = False
    zipline_length_loaded = False
    zipline_length = 0
    zipline_start = 0
    zipline_end = 0

    # Initialize peripherals
    leds = leds_ctrl.leds_init()
    odrv = motor_ctrl.motor_init()
    # ser = serial.Serial(config.SERIAL_PORT_LI550, config.BAUD_RATE_LI550, timeout=1)

    # Try to load calibration data from today's file
    calibration_data = calibration_file_handling.load_calibration_data()
    if calibration_data is not None:
        zipline_length = calibration_data
        zipline_end_set = True # Skip setting the end of the zipline in the calibration mode
        zipline_length_loaded = True
        print(f"Using saved calibration: length={zipline_length:.2f}m")
        leds_ctrl.leds_show_setpoint_calibration(leds) 
        time.sleep(5)  # Signal that we've loaded calibration

    # Motor settings for calibration
    odrv.axis0.pos_estimate = motor_ctrl.linear_to_angular(-config.INITIAL_MOTOR_POS_CALIB)
    odrv.axis0.trap_traj.config.vel_limit = motor_ctrl.linear_to_angular(config.MAX_SPEED_CALIB)  
    x_ref = config.INITIAL_MOTOR_POS_CALIB
    last_x_ref = config.INITIAL_MOTOR_POS_CALIB
    vel_ref = 0.0
    last_vel_ref = 0.0

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
                time_start_while = time.time()

                # Check if there is an error on the ODrive
                if (odrv.axis0.active_errors != 0 or (odrv.axis0.disarm_reason != 0)):
                    print(f"ODrive error: {odrv.axis0.active_errors}  Disarm reason: {odrv.axis0.disarm_reason}")
                    leds_off_before = leds_ctrl.leds_error_warning(leds, leds_off_before)
                    odrv.axis0.controller.input_pos = odrv.axis0.pos_estimate # stay in the same position
                else:
                    # Get remote control commands
                    with shared_remote_command.get_lock(), shared_target_speed.get_lock(), shared_calibration_setpoints.get_lock():
                        remote_command = shared_remote_command.value                # 0: no command, 1: go backward, 2: go forward, 3: go tracking
                        target_speed = shared_target_speed.value                    # in µs
                        calibration_setpoints = shared_calibration_setpoints.value  # 0: no setpoint, 1: zipline start, 2: zipline length

                        # Map target_speed (µs) to target_speed_m_s (m/s)
                        speed_max = config.MAX_SPEED_CALIB if in_calibration_mode else config.MAX_SPEED
                        if remote_command == 1:
                            target_speed_m_s = speed_max - np.interp(
                                target_speed,
                                [config.PWM_MIN_PULSE_WIDTH, config.PWM_DEFAULT_PULSE_WIDTH - config.GO_STOP_THRESHOLD],
                                [0.0, speed_max])
                        elif remote_command == 2:
                            target_speed_m_s = np.interp(
                                target_speed,
                                [config.PWM_DEFAULT_PULSE_WIDTH + config.GO_STOP_THRESHOLD, config.PWM_MAX_PULSE_WIDTH],
                                [0.0, speed_max])
                        else:
                            target_speed_m_s = 0.0
                    
                    # Get the ArUco marker position and the time at which the frame was captured
                    with shared_x_aruco.get_lock(), shared_y_aruco.get_lock(), shared_z_aruco.get_lock(), shared_roll_aruco.get_lock(), shared_pitch_aruco.get_lock(), shared_yaw_aruco.get_lock(), shared_time_frame_captured.get_lock():
                         tracking_error = shared_x_aruco.value
                         ArUco_pose = {      
                            'x ArUco [m]': tracking_error,
                            'y ArUco [m]': shared_y_aruco.value,
                            'z ArUco [m]': shared_z_aruco.value,
                            'roll ArUco [deg]': shared_roll_aruco.value,
                            'pitch ArUco [deg]': shared_pitch_aruco.value,
                            'yaw ArUco [deg]': shared_yaw_aruco.value,
                         }
                         time_frame_captured = shared_time_frame_captured.value                        
                    
                    # Get motor data                    
                    angular_position, angular_velocity, torque, linear_position, linear_velocity, voltage, current, decel_dist = motor_ctrl.get_data(odrv) # in turns, turns/s, Nm, m, m/s
                    time_position_measured = time.time()

                    # Get LI-5500 data
                    # li550_data = airspeed_sensor_ctrl.log_li550_data(ser)
                    li550_data = {}

                    # Update state
                    state = state_machine.update(last_state, remote_command, in_calibration_mode)
                    last_state = state

                    # Calibrating the zipline length
                    if in_calibration_mode:
                        print(f"CALIBRATION  Position: {linear_position:.2f} m  |  Velocity: {linear_velocity:.2f} m/s")

                        # Display the calibration mode with LEDs
                        leds_off_before = leds_ctrl.leds_set_color_calibration(leds, leds_off_before)
                        
                        # Setting the end of the zipline
                        if (calibration_setpoints == 2) and not zipline_end_set:
                            zipline_end = linear_position
                            zipline_end_set = True
                            print(f"Zipline end set: {zipline_end:.2f} m")
                            leds_ctrl.leds_show_setpoint_calibration(leds)
                            time.sleep(3)  # Signal that we've set the end of the line
                        
                        # Setting the start of the zipline
                        if (calibration_setpoints == 1) and zipline_end_set:
                            if (not zipline_length_loaded) and (zipline_start != zipline_end):
                                # If there is no calibration data, compute the zipline length from the start and end positions
                                zipline_start = linear_position
                                zipline_length = zipline_end - zipline_start

                            # Validate the calibration
                            if zipline_length <= 0:
                                print(f"ERROR: Invalid zipline length: {zipline_length:.2f} m. Start position must be less than end position.")
                                # Flash error pattern on LEDs
                                leds_ctrl.leds_error_warning(leds, True)
                                time.sleep(3)
                                # Reset calibration
                                zipline_end_set = False
                            else:
                                # Motor settings for normal operation
                                odrv.axis0.pos_estimate = motor_ctrl.linear_to_angular(0)
                                odrv.axis0.trap_traj.config.vel_limit = motor_ctrl.linear_to_angular(config.MAX_SPEED)
                                linear_position = 0
                                x_ref = 0
                                last_x_ref = 0                        
                                in_calibration_mode = False # Calibration done   

                                # Save calibration data to file
                                calibration_file_handling.save_calibration_data(zipline_length)
                                print(f"Calibration done: zipline length: {zipline_length:.2f} m")
                                leds_ctrl.leds_show_setpoint_calibration(leds)
                                time.sleep(3)  # Reduced wait time 

                        # Don't capture frames from the camera in this mode
                        with shared_detect_flag.get_lock():
                            shared_detect_flag.value = 0
                        
                        # Update the target position of the motor
                        if state == config.STATE["STOP"]:
                            x_ref = linear_position
                    
                        elif state == config.STATE["FORWARD"]:
                            x_ref = linear_position + target_speed_m_s * config.DT * config.BANG_BANG_GAIN_CALIB

                        elif state == config.STATE["BACKWARD"]:
                            x_ref = linear_position - target_speed_m_s * config.DT * config.BANG_BANG_GAIN_CALIB

                        else:
                            x_ref = linear_position
                        
                        # Set the target position of the motor
                        motor_ctrl.set_position(odrv, x_ref)

                    # Normal operation
                    else:
                        if state == config.STATE["TRACKING"]:
                            # Start camera recording and get ArUco marker position
                            with shared_detect_flag.get_lock():
                                shared_detect_flag.value = 1

                            if np.isnan(tracking_error):
                                tracking_error = None
                            if tracking_error is not None:
                                # ArUco marker detected: compute the new target position

                                estimated_position = linear_position + tracking_error + linear_velocity * config.DT * config.prediction_step
                                target_position = linear_position + tracking_error
                                cnt_moving_blindly = 0
        
                                if last_time_frame_captured is not None:
                                    time_diff = time_frame_captured - last_time_frame_captured

                                    estimated_velocity = (target_position - last_target_position) / time_diff if time_diff > 0 else 0
                                    estimated_velocity = motor_ctrl.low_pass(estimated_velocity, last_estimated_velocity, config.CUT_OFF_FREQUENCY_TRACKING, config.DT) # low pass filter

                                    
                                    last_estimated_velocity = estimated_velocity

                                    # print("time_diff", time_diff, "estimated_velocity", estimated_velocity)
                                last_time_frame_captured = time_frame_captured
                                last_tracking_error = tracking_error
                                last_target_position = target_position

                                vel_ref = estimated_velocity + tracking_error * config.BANG_BANG_VISION
                                print("tag", estimated_velocity, tracking_error * config.BANG_BANG_VISION)

                                # if ((abs(linear_velocity) < 0.5) & (abs(tracking_error) < 0.25)):
                                #     x_ref = linear_position
                                if (vel_ref > 0):
                                    x_ref = zipline_length
                                elif (vel_ref == 0):
                                    x_ref = linear_position
                                else:
                                    x_ref = 0

                                vel_ref = np.clip(abs(vel_ref), 0, config.MAX_SPEED)
                                odrv.axis0.trap_traj.config.vel_limit = motor_ctrl.linear_to_angular(vel_ref)  
                            else:
                                # ArUco marker not detected: keep the last tracking_error for a while
                                if (last_tracking_error is not None) and cnt_moving_blindly < config.MAX_CNT_MOVING_BLINDLY:
                                    # x_ref = (linear_position + last_tracking_error) + last_estimated_velocity * config.DT * config.prediction_step # where we think we are
                                    # estimated_position = x_ref
                                    # x_ref += np.clip(last_estimated_velocity * config.BANG_BANG_GAIN_TRACKING, -config.MAX_SPEED, config.MAX_SPEED) * config.DT # to catch up
                                    if (last_estimated_velocity > 0):
                                        x_ref = zipline_length
                                    elif (last_estimated_velocity == 0):
                                        x_ref = linear_position
                                    else:
                                        x_ref = 0
                                    
                                    vel_ref = last_estimated_velocity * config.BANG_BANG_GAIN_TRACKING
                                    print("lost", vel_ref)
                                    vel_ref = np.clip(abs(vel_ref), 0, config.MAX_SPEED)
                                    odrv.axis0.trap_traj.config.vel_limit = motor_ctrl.linear_to_angular(vel_ref)  
                                    
                                    cnt_moving_blindly += 1
                                else:
                                    # No ArUco marker detected and no previous offset: stop the motor
                                    x_ref = linear_position
                                    odrv.axis0.trap_traj.config.vel_limit = motor_ctrl.linear_to_angular(config.MAX_SPEED)
                                    estimated_position = x_ref
                            
                            start_decelerating = True

                        else:
                            with shared_detect_flag.get_lock():
                                shared_detect_flag.value = 1
                            tracking_error = None
                            last_tracking_error = None

                            # Desired velocity based on the remote command
                            vel_ref = target_speed_m_s

                            if state == config.STATE["STOP"]:
                                # Define positon target to match desired deceleration with respect to current velocity
                                if start_decelerating:
                                    if linear_velocity > 0:
                                        x_stop = linear_position + decel_dist
                                    else:
                                        x_stop = linear_position - decel_dist
                                    start_decelerating = False

                                # Stay on spot when position target for deceleration reached   
                                decelerating = True
                                if abs(linear_velocity) < config.STOP_SPEED_THRESHOLD: decelerating = False
                                if decelerating:
                                    x_ref = x_stop
                                else:
                                    x_ref = linear_position
                                                            
                            elif state == config.STATE["FORWARD"]:
                                x_ref = zipline_length
                                start_decelerating = True

                            elif state == config.STATE["BACKWARD"]:
                                x_ref = 0
                                start_decelerating = True

                            else:
                                x_ref = linear_position
                                start_decelerating = True

                            # To switch from FORWARD to TRACKING smoothly
                            last_estimated_velocity = linear_velocity

                        # Set the target position and velocity of the motor
                        x_ref = np.clip(x_ref, 0, zipline_length) # important safety
                        motor_ctrl.set_velocity(odrv, vel_ref)
                        motor_ctrl.set_position(odrv, x_ref)

                        last_x_ref = x_ref
                        last_vel_ref = vel_ref

                        # Display current state with LEDs
                        leds_off_before = leds_ctrl.leds_set_color(leds, state, tracking_error, leds_off_before, in_calibration_mode)

                        # Save data to CSV
                        motor_data = motor_ctrl.log_motor_data(time.time() - time_start_ref, angular_position, angular_velocity, torque, linear_position, linear_velocity, tracking_error, voltage, current, x_ref, estimated_position, estimated_velocity)
                        row = {**motor_data, **ArUco_pose, **li550_data}
                        writer.writerow(row)   
                        csv_file.flush()

                        # Print current state
                        offset_str = "N/A" if tracking_error is None else f"{tracking_error:.2f} m"
                        log_message = (f"Cmd: {config.COMMAND_LOOKUP.get(remote_command, 'UNKNOWN')}  |  "
                                       f"State: {config.STATE_LOOKUP.get(state, 'UNKNOWN')}  |   "
                                       f"Pos: {linear_position:.2f} m  |  "
                                       f"Vel: {linear_velocity:.2f} m/s  |  "
                                       f"ArUco offset: {offset_str} m  |  "
                                       f"x_ref: {x_ref} m")
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
            odrv.axis0.requested_state = 1  # Set ODrive to idle state
            leds.fill((0, 0, 0)) # Turn off LEDs
            leds.show()

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
    time_start_ref = time.time()

    # Shared variables from RC process to main process
    shared_remote_command = Value('i', 0)
    shared_target_speed = Value('d', 0.0)
    shared_calibration_setpoints = Value('d', 0.0)

    # Shared variables between camera process and main process
    shared_x_aruco = Value('d', float('nan'))
    shared_y_aruco = Value('d', float('nan'))
    shared_z_aruco = Value('d', float('nan'))
    shared_roll_aruco = Value('d', float('nan'))
    shared_pitch_aruco = Value('d', float('nan'))
    shared_yaw_aruco = Value('d', float('nan'))
    shared_time_frame_captured = Value('d', 0.0)
    shared_detect_flag = Value('i', 0)

    p1 = Process(target=rc_receiver_reading, args=(shared_remote_command, shared_target_speed, shared_calibration_setpoints))
    p2 = Process(target=camera_process, args=(save_path, time_start_ref, shared_x_aruco, shared_y_aruco, shared_z_aruco, shared_roll_aruco, shared_pitch_aruco, shared_yaw_aruco, shared_time_frame_captured, shared_detect_flag))
    p3 = Process(target=main, args=(save_path, time_start_ref, shared_remote_command, shared_target_speed, shared_calibration_setpoints, shared_x_aruco, shared_y_aruco, shared_z_aruco, shared_roll_aruco, shared_pitch_aruco, shared_yaw_aruco, shared_time_frame_captured, shared_detect_flag))

    p1.start()
    p2.start()
    p3.start()

    p1.join()
    p2.join()
    p3.join()