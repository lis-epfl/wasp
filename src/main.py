import time
from multiprocessing import Process, Value
from collections import deque
import gpiod
from picamera2.encoders import H264Encoder, Quality
from picamera2.outputs import FfmpegOutput
from datetime import datetime
import pandas as pd
import numpy as np

import config
import state_machine
import leds_ctrl
import ultrasonic_ctrl
import motor_ctrl
import camera_ctrl
import rc_receiver_ctrl


def rc_receiver_reading(shared_val1, shared_val2):
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
                        if (button_position != last_button_position) or (throttle_pulse > (config.PWM_DEFAULT_PULSE_WIDTH + config.GO_STOP_TRHESHOLD)) or (throttle_pulse < (config.PWM_DEFAULT_PULSE_WIDTH - config.GO_STOP_TRHESHOLD)):
                            # if the button or the trottle is touched, stop tracking
                            remote_command = 0 # corresponding to "GO_STOP"
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
                with shared_val1.get_lock():
                    shared_val1.value = remote_command
                with shared_val2.get_lock():
                    shared_val2.value = target_speed

                time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        trottle_line.release()
        button_line.release()
        chip.close()


def main(shared_val1, shared_val2):
    # Initialize peripherals
    leds = leds_ctrl.leds_init()
    odrv = motor_ctrl.motor_init()
    # camera = camera_ctrl.camera_init()

    # Caamera recording setings 
    save_path = "data"
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    # filename = f"{save_path}/video_{timestamp}.mp4"
    # encoder = H264Encoder()
    # output = FfmpegOutput(filename)

    # Time variables
    time_start_while = 0
    time_end_while = 0
    time_start_abs = time.time()

    # Initial state
    state = config.STATE["STOP"]
    last_state = state

    # Deques to store the last ultrasonic sensor readings
    front_readings = deque(maxlen=config.NB_READINGS)
    back_readings = deque(maxlen=config.NB_READINGS)

    want_to_stop = False

    leds_off_before = False

    # Data collection
    columns = [
        "run_time (s)",
        "angular_position (turns)",
        "angular_velocity (turns/s)",
        "torque (Nm)",
        "linear_position (m)",
        "linear_speed (m/s)"
    ]
    df = pd.DataFrame(columns=columns)
    csv_path = f"{save_path}/data_{timestamp}.csv"
    df.to_csv(csv_path, index=False)

    try:
        # Start camera recording
        #print("Recording started:")
        #camera.start_recording(encoder, output, quality=Quality.HIGH)
        
        while True:
            if odrv.axis0.active_errors != 0:
                # If error on the motor detected, then blink yellow and stop everything 
                print("Error with the motor:", odrv.axis0.active_errors)
                leds_off_before = leds_ctrl.leds_error_warning(leds, leds_off_before)
                state = config.STATE["STOP"]
                target_speed = 0
                motor_ctrl.set_cart_velocity(odrv, state, target_speed)
                time.sleep(config.DT)
                continue
                
            else:
                # Normal operation
                time_start_while = time.time()

                # Record data
                current_angular_position, current_angular_velocity, current_torque = motor_ctrl.get_data(odrv) # in turns, turns/s, Nm
                current_linear_position = motor_ctrl.compute_linear_position(current_angular_position) # in m
                current_linear_velocity = motor_ctrl.compute_linear_speed(current_angular_velocity) # in m/s
                current_run_time = time.time() - time_start_abs

                new_data = pd.DataFrame([[
                    current_run_time,
                    current_angular_position,
                    current_angular_velocity,
                    current_torque,
                    current_linear_position,
                    current_linear_velocity
                ]], columns=columns)
                new_data.to_csv(csv_path, mode='a', header=False, index=False)

                if want_to_stop:
                    if odrv.axis0.vel_estimate < config.STOP_SPEED_THRESHOLD:
                        # Motor is stopped
                        want_to_stop = False
                else:
                    # Get remote control commands
                    with shared_val1.get_lock():
                        remote_command = shared_val1.value
                    with shared_val2.get_lock():
                        target_speed = shared_val2.value

                    # Get distance sensor readings
                    front_value = ultrasonic_ctrl.get_distance(config.PIN_FRONT)
                    back_value = ultrasonic_ctrl.get_distance(config.PIN_BACK)
                    # print('Back: {:.2f} m    |    Front: {:.2f} m'.format(back_value, front_value))

                    front_readings.append(front_value)
                    back_readings.append(back_value)

                    # Determine obstacle presence
                    obstacle_forward = len(front_readings) == config.NB_READINGS and all(d <= config.OBST_THRESHOLD for d in front_readings)
                    obstacle_backward = len(back_readings) == config.NB_READINGS and all(d <= config.OBST_THRESHOLD for d in back_readings)

                    # Update state
                    state = state_machine.update(last_state, remote_command, obstacle_forward, obstacle_backward, current_linear_position, current_angular_velocity)
                    last_state = state

                    if state == config.STATE["STOP"]:
                        # Stop the motor
                        want_to_stop = True

                # Print current state
                log_message = f"Command: {config.COMMAND_LOOKUP.get(remote_command, 'UNKNOWN')}  |  Target velocity: {target_speed:.2f}  |  " \
                              f"Backward obst.: {obstacle_backward}  |  Forward obst.: {obstacle_forward}  |  " \
                              f"State: {config.STATE_LOOKUP.get(state, 'UNKNOWN')}  |   " \
                              f"Position: {current_linear_position:.2f} m  |  " \
                              f"Velocity: {current_linear_velocity:.2f} m/s"
                print(log_message)

                # Display current state with LEDs
                leds_off_before = leds_ctrl.leds_set_color(leds, state, obstacle_forward, obstacle_backward, leds_off_before)

                # Set motor velocity based on state
                motor_ctrl.set_cart_velocity(odrv, state, target_speed)

                # Sleep to respect the desired loop time
                
                time_end_while = time.time()
                #print(time_end_while - time_start_while)
                
                if time_end_while - time_start_while < config.DT:
                    time.sleep(config.DT - (time_end_while - time_start_while))
                else:
                    print(f"Execution time exceeded {config.DT} seconds.")
    
    except KeyboardInterrupt:
        print("\nStopping recording...")
        # camera.stop_recording()
        # print(f"Video saved as {filename}")


if __name__ == "__main__":
    shared_val1 = Value('i', 0)  # remote_command state sent by the remote control
    shared_val2 = Value('d', 0.0) # target_speed state sent by the remote control

    p1 = Process(target=rc_receiver_reading, args=(shared_val1, shared_val2))
    p2 = Process(target=main, args=(shared_val1, shared_val2))

    p1.start() # Start RC receiver process
    p2.start() # Start entire system process

    p1.join()
    p2.join()