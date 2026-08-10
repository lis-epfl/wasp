import lgpio
import numpy as np

import config


def servo_init():
    """
    Connect to the servo and move it to its start position
    :return servo: GPIO chip handle if found, None otherwise
    """
    try:
        print("Opening servo GPIO...")
        servo = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(servo, config.SERVO_PIN)
        print(f"Servo GPIO {config.SERVO_PIN} claimed, moving to start position")

        set_position(servo, config.SERVO_START_POSITION)

        return servo

    except Exception as e:
        print(f"Error: {e}")
        return None


def position_to_pulse_width(position):
    """
    Convert a normalized position to a servo pulse width
    :param position: Position between -1.0 and 1.0
    :return pulse_width: Pulse width in µs
    """
    position = np.clip(position, -1.0, 1.0)
    pulse_width = int(500 + (position + 1) * 1000)      # -1.0 to 1.0 -> 500 µs to 2500 µs
    return pulse_width


def set_position(servo, position):
    """
    Set the position of the servo (non blocking, the pulses keep being sent until the next call)
    :param servo: GPIO chip handle
    :param position: Position between -1.0 and 1.0
    """
    if servo is None:
        return

    lgpio.tx_servo(servo, config.SERVO_PIN, position_to_pulse_width(position))


def servo_deinit(servo):
    """
    Stop sending pulses to the servo and release the GPIO pin
    :param servo: GPIO chip handle
    """
    if servo is None:
        return

    lgpio.tx_servo(servo, config.SERVO_PIN, 0)
    lgpio.gpio_free(servo, config.SERVO_PIN)
    lgpio.gpiochip_close(servo)
