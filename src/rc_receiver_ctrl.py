import gpiod
import numpy as np
import time

import config


def define_command(throttle_pulse, button_pulse, throttle_timeout, button_timeout, remote_command, last_remote_command, target_speed, button_position, last_button_position):

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

    return remote_command, target_speed