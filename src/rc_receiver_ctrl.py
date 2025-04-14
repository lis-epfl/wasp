import gpiod
import numpy as np
import time

import config


with gpiod.Chip('gpiochip0') as chip:
    line = chip.get_line(config.TROTTLE_PIN)
    line.request(consumer='pwm_reader', type=gpiod.LINE_REQ_EV_BOTH_EDGES)

    last_rising = None

    while True:
        event = line.event_wait(sec=1)
        if event:
            ev = line.event_read()
            timestamp = ev.sec + ev.nsec / 1e9

            if ev.type == gpiod.LineEvent.RISING_EDGE:
                last_rising = timestamp
            elif ev.type == gpiod.LineEvent.FALLING_EDGE and last_rising is not None:
                pulse_width = (timestamp - last_rising) * 1_000_000  # in µs

                if (pulse_width < (config.PWM_DEFAULT_PULSE_WIDTH + config.GO_STOP_TRHESHOLD)) and (pulse_width > (config.PWM_DEFAULT_PULSE_WIDTH - config.GO_STOP_TRHESHOLD)):
                    remote_command = config.REMOTE_COMMAND["GO_STOP"]
                    target_speed = 0.0

                elif pulse_width < config.PWM_DEFAULT_PULSE_WIDTH:
                    remote_command = config.REMOTE_COMMAND["GO_BACKWARD"]
                    target_speed = config.MANUAL_MOTOR_SPEED - np.interp(pulse_width, [config.PWM_MIN_PULSE_WIDTH, config.PWM_DEFAULT_PULSE_WIDTH], [0.0, config.MANUAL_MOTOR_SPEED])

                elif pulse_width > config.PWM_DEFAULT_PULSE_WIDTH:
                    remote_command = config.REMOTE_COMMAND["GO_FORWARD"]
                    target_speed = np.interp(pulse_width, [config.PWM_DEFAULT_PULSE_WIDTH, config.PWM_MAX_PULSE_WIDTH], [0.0, config.MANUAL_MOTOR_SPEED])

                print(f"Pulse width: {pulse_width:.1f} µs {config.COMMAND_LOOKUP.get(remote_command, 'UNKNOWN')} {target_speed:.2f} m/s")
                last_rising = None
        else:
            print("Timeout waiting for edge...")