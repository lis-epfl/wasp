import gpiod
import numpy as np
import time

import config


with gpiod.Chip('gpiochip0') as chip:
    trottle_line = chip.get_line(config.TROTTLE_PIN)
    button_line = chip.get_line(config.BUTTON_PIN)

    trottle_line.request(consumer='pwm_reader', type=gpiod.LINE_REQ_EV_BOTH_EDGES)
    button_line.request(consumer='pwm_reader', type=gpiod.LINE_REQ_EV_BOTH_EDGES)

    # Initialize variables for both pins
    last_rising_1 = None
    last_rising_2 = None
    line1_timeout = False
    line2_timeout = False
    throttle_pulse = 0.0
    button_pulse = 0.0
    remote_command = 0 
    target_speed = 0.0
    button_position = True
    button_position_prev = True

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
            print("Timeout on throttle...")
            throttle_pulse = 0.0
            line1_timeout = True

        if event_2:
            ev_2 = button_line.event_read()
            timestamp_2 = ev_2.sec + ev_2.nsec / 1e9

            if ev_2.type == gpiod.LineEvent.RISING_EDGE:
                last_rising_2 = timestamp_2
            elif ev_2.type == gpiod.LineEvent.FALLING_EDGE and last_rising_2 is not None:
                button_pulse = (timestamp_2 - last_rising_2) * 1_000_000  # in µs
                last_rising_2 = None
        else:
            print("Timeout on button...")
            button_pulse = 0.0
            line2_timeout = True

        if line1_timeout or line2_timeout:
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
                if (button_position != button_position_prev) or (throttle_pulse > (config.PWM_DEFAULT_PULSE_WIDTH + config.GO_STOP_TRHESHOLD)) or (throttle_pulse < (config.PWM_DEFAULT_PULSE_WIDTH - config.GO_STOP_TRHESHOLD)):
                    # if the button or the trottle is touched, stop tracking
                    remote_command = 0
                    target_speed = 0.0
            else:
                # Manual mode
                if button_position != button_position_prev:
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
                    
            button_position_prev = button_position

            print(f" {button_position} Before sending: {config.COMMAND_LOOKUP.get(remote_command, 'UNKNOWN')} {target_speed:.2f} m/s")
