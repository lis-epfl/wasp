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

import config
import state_machine
import leds_ctrl
import ultrasonic_ctrl
import motor_ctrl
import camera_ctrl
import plot_velocity_pos_torque


def rc_receiver_reading(shared_remote_command, shared_target_speed):
    try:
        with gpiod.Chip('gpiochip0') as chip:
            trottle_line = chip.get_line(config.TROTTLE_PIN)
            button_line = chip.get_line(config.BUTTON_PIN)

            trottle_line.request(consumer='pwm_reader', type=gpiod.LINE_REQ_EV_BOTH_EDGES)
            button_line.request(consumer='pwm_reader', type=gpiod.LINE_REQ_EV_BOTH_EDGES)

            # Initialize variables for both pins
            last_rising_1 = None
            throttle_timeout = False
            throttle_pulse = 0.0

            last_rising_2 = None
            button_timeout = False
            button_pulse = 0.0
            
            remote_command = 0
            last_remote_command = 0
            target_speed = 0.0
            button_position = True
            last_button_position = True

            while True:
                event_1 = trottle_line.event_wait(sec=1)
                event_2 = button_line.event_wait(sec=1)

                if event_1:
                    ev_1 = trottle_line.event_read()
                    timestamp_1 = ev_1.sec + ev_1.nsec / 1e9
                    if ev_1.type == gpiod.LineEvent.RISING_EDGE:
                        last_rising_1 = timestamp_1
                    elif ev_1.type == gpiod.LineEvent.FALLING_EDGE and last_rising_1 is not None:
                        throttle_pulse = (timestamp_1 - last_rising_1) * 1_000_000  # in µs
                        last_rising_1 = None
                else:
                    throttle_pulse = 0.0
                    throttle_timeout = True

                if event_2:
                    ev_2 = button_line.event_read()
                    timestamp_2 = ev_2.sec + ev_2.nsec / 1e9
                    if ev_2.type == gpiod.LineEvent.RISING_EDGE:
                        last_rising_2 = timestamp_2
                    elif ev_2.type == gpiod.LineEvent.FALLING_EDGE and last_rising_2 is not None:
                        button_pulse = (timestamp_2 - last_rising_2) * 1_000_000  # in µs
                        last_rising_2 = None
                else:
                    button_pulse = 0.0
                    button_timeout = True

                # Logic to define the command based on the pulse widths and eventual timeouts
                if throttle_timeout or button_timeout:
                    if last_remote_command == 3:
                        remote_command = 3  # stay in "GO_TRACKING" if deconnection
                        target_speed = 0.0
                    else:
                        remote_command = 0  # corresponding to "GO_STOP"
                        target_speed = 0.0
                else:
                    # If pulse width is low, button is on its first position, else it is on its second position
                    if button_pulse < config.PWM_DEFAULT_PULSE_WIDTH:
                        button_position = True
                    else:
                        button_position = False

                    if remote_command == 3:
                        # Tracking mode
                        if (button_position != last_button_position) or (throttle_pulse > (config.PWM_DEFAULT_PULSE_WIDTH + config.STAY_TRACKING_TRHESHOLD)) or (throttle_pulse < (config.PWM_DEFAULT_PULSE_WIDTH - config.STAY_TRACKING_TRHESHOLD)):
                            # if the button or the trottle is touched, stop tracking
                            remote_command = 0 # corresponding to "GO_STOP"
                            target_speed = 0.0
                        else:
                            # if the button is not touched, keep tracking
                            remote_command = 3
                            target_speed = 0.0
                    else:
                        # Manual mode
                        if button_position != last_button_position:
                            remote_command = 3 # corresponding to "GO_TRACKING"
                            target_speed = 0.0
                        else:
                            if (throttle_pulse < (config.PWM_DEFAULT_PULSE_WIDTH + config.GO_STOP_TRHESHOLD)) and (throttle_pulse > (config.PWM_DEFAULT_PULSE_WIDTH - config.GO_STOP_TRHESHOLD)):
                                remote_command = 0  # corresponding to "GO_STOP"
                                target_speed = 0.0
                            elif throttle_pulse < config.PWM_DEFAULT_PULSE_WIDTH:
                                remote_command = 1  # corresponding to "GO_BACKWARD"
                                target_speed = config.MANUAL_MOTOR_SPEED - np.interp(throttle_pulse, [config.PWM_MIN_PULSE_WIDTH, config.PWM_DEFAULT_PULSE_WIDTH], [0.0, config.MANUAL_MOTOR_SPEED])
                            elif throttle_pulse > config.PWM_DEFAULT_PULSE_WIDTH:
                                remote_command = 2  # corresponding to "GO_FORWARD"
                                target_speed = np.interp(throttle_pulse, [config.PWM_DEFAULT_PULSE_WIDTH, config.PWM_MAX_PULSE_WIDTH], [0.0, config.MANUAL_MOTOR_SPEED])
                last_button_position = button_position
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
        trottle_line.release()
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

    # Variable initialization
    time_start_while = 0
    time_end_while = 0
    time_start_abs = time.time()
    want_to_stop = False
    leds_off_before = False
    state = config.STATE["STOP"]
    last_state = state
    front_readings = deque(maxlen=config.NB_READINGS)
    back_readings = deque(maxlen=config.NB_READINGS)
    offset_from_center = None
    last_offset_from_center = None
    cnt_moving_blindly = 0
    pid = PID(config.KP, config.KI, config.KD, setpoint=0)
    pid.output_limits = (-config.MAX_TRACKING_SPEED, config.MAX_TRACKING_SPEED)
    pid.sample_time = config.DT
    pid.auto_mode = True

    # Data recording setings 
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    columns = [
        "run_time (s)",
        "angular_position (turns)",
        "angular_velocity (turns/s)",
        "torque (Nm)",
        "linear_position (m)",
        "linear_speed (m/s)"
    ]
    csv_path = Path(save_path) / f"data_{timestamp}.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)  # Create dir if missing
    csv_file = open(csv_path, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(columns)
    
    try:
        try:
            while True:
                if odrv.axis0.active_errors != 0:
                    # If error on the motor detected, then blink yellow and stop everything 
                    print("Error with the motor:", odrv.axis0.active_errors)

                    leds_off_before = leds_ctrl.leds_error_warning(leds, leds_off_before)
                    motor_ctrl.set_cart_velocity(odrv, config.STATE["STOP"], 0)
                    time.sleep(config.DT)
                else:
                    # Normal operation
                    time_start_while = time.time()

                    # Record data
                    current_angular_position, current_angular_velocity, current_torque = motor_ctrl.get_data(odrv) # in turns, turns/s, Nm
                    current_linear_position = motor_ctrl.compute_linear_position(current_angular_position) # in m
                    current_linear_velocity = motor_ctrl.compute_linear_speed(current_angular_velocity) # in m/s
                    current_run_time = time.time() - time_start_abs

                    csv_writer.writerow([
                        current_run_time,
                        current_angular_position,
                        current_angular_velocity,
                        current_torque,
                        current_linear_position,
                        current_linear_velocity
                    ])

                    if want_to_stop:
                        # Decelerate the motor until it stops
                        if odrv.axis0.vel_estimate < config.STOP_SPEED_THRESHOLD:
                            # Motor is now stopped
                            want_to_stop = False
                    else:
                        # Get remote control commands
                        with shared_remote_command.get_lock(), shared_target_speed.get_lock():
                            remote_command = shared_remote_command.value
                            target_speed = shared_target_speed.value

                        # Get distance sensor readings
                        front_value = ultrasonic_ctrl.get_distance(config.PIN_FRONT)
                        back_value = ultrasonic_ctrl.get_distance(config.PIN_BACK)
                        # print('Back: {:.2f} m    |    Front: {:.2f} m'.format(back_value, front_value))
                        front_readings.append(front_value)
                        back_readings.append(back_value)
                        obstacle_forward = len(front_readings) == config.NB_READINGS and all(d <= config.OBST_THRESHOLD for d in front_readings)
                        obstacle_backward = len(back_readings) == config.NB_READINGS and all(d <= config.OBST_THRESHOLD for d in back_readings)

                        # Update state
                        state = state_machine.update(last_state, remote_command, obstacle_forward, obstacle_backward, current_linear_position, current_angular_velocity)
                        last_state = state

                        if state == config.STATE["STOP"]:
                            # Stop the motor
                            want_to_stop = True
                        
                        if state == config.STATE["TRACKING"]:
                            # Start camera recording and get ArUco marker position
                            with shared_detect_flag.get_lock():
                                shared_detect_flag.value = 1
                            with shared_offset.get_lock():
                                offset_from_center = shared_offset.value

                            offset_from_center = offset_from_center if not np.isnan(offset_from_center) else None

                            if offset_from_center is not None:
                                # ArUco marker detected: compute target speed using PID controller
                                target_speed = pid(offset_from_center)
                                cnt_moving_blindly = 0
                            else:
                                # ArUco marker not detected: keep the last target speed for a while
                                if (last_offset_from_center is not None) and cnt_moving_blindly < config.MAX_CNT_MOVING_BLINDLY:
                                    target_speed = pid(last_offset_from_center)
                                    cnt_moving_blindly += 1
                                else:
                                    # No ArUco marker detected and no previous offset: stop the motor
                                    target_speed = 0.0
                            
                            last_offset_from_center = offset_from_center
                        else:
                            with shared_detect_flag.get_lock():
                                shared_detect_flag.value = 0
                            offset_from_center = None

                    # Display current state with LEDs
                    leds_off_before = leds_ctrl.leds_set_color(leds, state, obstacle_forward, obstacle_backward, offset_from_center, leds_off_before)

                    # Set motor velocity based on state
                    motor_ctrl.set_cart_velocity(odrv, state, target_speed)

                    # Print current state
                    offset_str = "N/A" if offset_from_center is None else f"{offset_from_center:.2f} m"
                    log_message = (f"Command: {config.COMMAND_LOOKUP.get(remote_command, 'UNKNOWN')}  |  "
                                   f"Backward obst.: {obstacle_backward}  |  Forward obst.: {obstacle_forward}  |  "
                                   f"State: {config.STATE_LOOKUP.get(state, 'UNKNOWN')}  |   "
                                   f"Position: {current_linear_position:.2f} m  |  "
                                   f"Velocity: {current_linear_velocity:.2f} m/s\n"
                                   f"ArUco position: {offset_str} m  |  Target velocity: {target_speed:.2f} m/s")
                    # print(log_message)

                    # Sleep to respect the desired loop time
                    time_end_while = time.time()
                    # print("Main process:", time_end_while - time_start_while, "s") # 0.05s usually
                    if time_end_while - time_start_while < config.DT:
                        time.sleep(config.DT - (time_end_while - time_start_while))
                    else:
                        print(f"Main process: Execution time exceeded {config.DT} seconds.")

        except KeyboardInterrupt:
            print("\nMain process stopped.")
            motor_ctrl.set_cart_velocity(odrv, config.STATE["STOP"], 0)
            leds_off_before = leds_ctrl.leds_set_color(leds, config.STATE["STOP"], obstacle_forward, obstacle_backward, offset_from_center, leds_off_before)

    finally:
        csv_file.close()
        print(f"\nRun complete. Data saved to {csv_path}")
        try:
            plot_velocity_pos_torque.plot_data(csv_path)
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