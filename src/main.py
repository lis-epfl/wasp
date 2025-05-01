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


def rc_receiver_reading(shared_remote_command, shared_target_speed):
    try:
        with gpiod.Chip('gpiochip0') as chip:
            throttle_line = chip.get_line(config.TROTTLE_PIN)
            button_line = chip.get_line(config.BUTTON_PIN)

            throttle_line.request(consumer='pwm_reader', type=gpiod.LINE_REQ_EV_BOTH_EDGES)
            button_line.request(consumer='pwm_reader', type=gpiod.LINE_REQ_EV_BOTH_EDGES)

            # Initialize variables for both pins
            last_rising_throttle = None
            throttle_timeout = False
            throttle_pulse = 0.0

            last_rising_button = None
            button_timeout = False
            button_pulse = 0.0
            
            remote_command = 0
            last_remote_command = 0
            target_speed = 0.0
            initial_button_pulse = 0.0
            button_in_default_position = True
            button_initialized = False

            while True:
                throttle_event = throttle_line.event_wait(sec=1)
                button_event = button_line.event_wait(sec=1)

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

                if throttle_timeout or button_timeout or (throttle_pulse == 0.0) or (button_pulse == 0.0):
                    if last_remote_command == 3:
                        remote_command = 3  # stay in "GO_TRACKING" if deconnection and was in tracking mode
                        target_speed = 0.0
                    else:
                        remote_command = 0 # chang to "GO_STOP" if deconnection and was in manual mode
                        target_speed = 0.0
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
                                target_speed = config.MAX_MANUAL_SPEED - np.interp(throttle_pulse, [config.PWM_MIN_PULSE_WIDTH, config.PWM_DEFAULT_PULSE_WIDTH], [0.0, config.MAX_MANUAL_SPEED])
                            elif throttle_pulse > (config.PWM_DEFAULT_PULSE_WIDTH + config.GO_STOP_THRESHOLD):
                                remote_command = 2  # corresponding to "GO_FORWARD"
                                target_speed = np.interp(throttle_pulse, [config.PWM_DEFAULT_PULSE_WIDTH, config.PWM_MAX_PULSE_WIDTH], [0.0, config.MAX_MANUAL_SPEED])
                            
                last_remote_command = remote_command

                # Update shared values between processes
                with shared_remote_command.get_lock():
                    shared_remote_command.value = remote_command
                with shared_target_speed.get_lock():
                    shared_target_speed.value = target_speed

                time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nRC remote process stopped.")
    finally:
        throttle_line.release()
        button_line.release()
        chip.close()


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


def main(save_path, shared_remote_command, shared_target_speed, shared_offset, shared_detect_flag):
    # Initialize peripherals
    leds = leds_ctrl.leds_init()
    odrv = motor_ctrl.motor_init()
    ser = serial.Serial(config.SERIAL_PORT_LI550, config.BAUD_RATE_LI550, timeout=1)

    # Variable initialization
    time_start_while = 0
    time_end_while = 0
    time_start_abs = time.time()
    want_to_stop = False
    leds_off_before = False
    state = config.STATE["STOP"]
    last_state = state
    tracking_error = None
    last_tracking_error = None
    cnt_moving_blindly = 0

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
                if odrv.axis0.active_errors != 0:
                    # If error on the motor detected, then blink yellow and stop everything 
                    print("Error with the motor:", odrv.a3xis0.active_errors) # (usually, it's 512 = DC_BUS_UNDER_VOLTAGE)

                    leds_off_before = leds_ctrl.leds_error_warning(leds, leds_off_before)
                    motor_ctrl.set_postion(odrv, odrv.axis0.pos_estimate) # stay in the same position
                    time.sleep(config.DT)
                else:
                    time_start_while = time.time()
                
                    # Read data from the LI-5500 sensor
                    li550_data = airspeed_sensor_ctrl.log_li550_data(ser)

                    # Read and save data from the ODrive motor
                    angular_position, angular_velocity, torque, linear_position, linear_velocity = motor_ctrl.get_data(odrv) # in turns, turns/s, Nm, m, m/s
                    
                    if want_to_stop:
                        # Decelerate the motor until it stops
                        if odrv.axis0.vel_estimate < config.STOP_SPEED_THRESHOLD:
                            # Motor is now stopped
                            want_to_stop = False
                    else:
                        # Get remote control commands
                        with shared_remote_command.get_lock(), shared_target_speed.get_lock():
                            remote_command = shared_remote_command.value
                            target_speed = shared_target_speed.value # in m/s

                        # Get distance sensor readings
                        obstacle_backward, obstacle_forward = ultrasonic_ctrl.is_there_an_obstacle()

                        # Update state
                        state = state_machine.update(last_state, remote_command, obstacle_forward, obstacle_backward, linear_position, angular_velocity)
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
                                x_ref = linear_position + tracking_error
                                cnt_moving_blindly = 0
                            else:
                                # ArUco marker not detected: keep the last target speed for a while
                                if (last_tracking_error is not None) and cnt_moving_blindly < config.MAX_CNT_MOVING_BLINDLY:
                                    tracking_error = last_tracking_error
                                    x_ref = linear_position + tracking_error
                                    cnt_moving_blindly += 1
                                else:
                                    # No ArUco marker detected and no previous offset: stop the motor
                                    x_ref = linear_position
                            
                            last_tracking_error = tracking_error
                        else:
                            with shared_detect_flag.get_lock():
                                shared_detect_flag.value = 0
                            tracking_error = None
                            last_tracking_error = None

                            if state == config.STATE["STOP"]:
                                # Stop the motor
                                want_to_stop = True
                                x_ref = linear_position # stay in the same position
                            elif state == config.STATE["FORWARD"]:
                                # Move forward
                                x_ref = linear_position + target_speed * config.DT
                            elif state == config.STATE["BACKWARD"]:
                                # Move backward
                                x_ref = linear_position - target_speed * config.DT
                            else:
                                # Stay in the same position
                                x_ref = linear_position

                    # Set motor velocity based on state
                    motor_ctrl.set_postion(odrv, motor_ctrl.compute_angular_position(x_ref))

                    # Display current state with LEDs
                    leds_off_before = leds_ctrl.leds_set_color(leds, state, obstacle_forward, obstacle_backward, tracking_error, leds_off_before)

                    # Save data to CSV
                    motor_data = motor_ctrl.log_motor_data(time.time() - time_start_abs, angular_position, angular_velocity, torque, linear_position, linear_velocity, tracking_error)
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
                                   f"ArUco position: {offset_str} m  |  Target velocity: {target_speed:.2f} m/s")
                    # print(log_message)

                    # Sleep to respect the desired loop time
                    time_end_while = time.time()
                    print("Main process:", time_end_while - time_start_while, "s") # 0.05s usually
                    if time_end_while - time_start_while < config.DT:
                        time.sleep(config.DT - (time_end_while - time_start_while))
                    else:
                        print(f"Main process: Execution time exceeded {config.DT} seconds.")

        except KeyboardInterrupt:
            print("\nMain process stopped.")
            motor_ctrl.set_cart_velocity(odrv, config.STATE["STOP"], 0)
            leds_off_before = leds_ctrl.leds_set_color(leds, config.STATE["STOP"], obstacle_forward, obstacle_backward, tracking_error, leds_off_before)

    finally:
        csv_file.close()
        print(f"\nRun complete. Data saved to {csv_path}")
        try:
            plot_motor_data.plot_data(csv_path)
            plot_li550_data.plot_data(csv_path)
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

    shared_offset = Value('d', float('nan'))  # ArUco detection result
    shared_detect_flag = Value('i', 0)        # flag to compute detection

    p1 = Process(target=rc_receiver_reading, args=(shared_remote_command, shared_target_speed))
    p2 = Process(target=camera_process, args=(save_path, shared_offset, shared_detect_flag))
    p3 = Process(target=main, args=(save_path, shared_remote_command, shared_target_speed, shared_offset, shared_detect_flag))

    p1.start()
    p2.start()
    p3.start()

    p1.join()
    p2.join()
    p3.join()