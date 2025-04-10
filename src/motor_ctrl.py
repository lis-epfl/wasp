import simple_pid as pid
import numpy as np
import odrive
import curses
import time
import os

import config


def compute_linear_speed(angular_speed):
    """
    Compute linear speed from angular speed
    :param angular_speed: Angular speed in turns/s
    :return: Linear speed in m/s
    """
    linear_speed = config.PULLEY_RADIUS * (2 * np.pi * angular_speed) # radius * angular speed
    return linear_speed


def compute_linear_position(angular_position):
    """
    Compute linear position from angular position
    :param angular_position: Angular position in turns
    :return: Linear position in m
    """
    linear_position = config.PULLEY_RADIUS * (2 * np.pi * angular_position) # radius * angle
    return linear_position


def motor_init():
    """
    Connect to ODrive and return the ODrive object
    :return: ODrive object if found, None otherwise
    """
    try:
        print("Searching for ODrive...")
        odrv = odrive.find_any(timeout=20)  # Timeout in seconds

        if odrv is None:
            print("No ODrive found!")

        else:
            print("ODrive found!")
            print("ODrive's configuration:")
            print(odrv)  # will print the motor's configuration

            # Depending on the battery
            odrv.config.dc_bus_undervoltage_trip_level = config.MIN_VOLTAGE
            odrv.config.dc_bus_overvoltage_trip_level = config.MAX_VOLTAGE
            
            odrv.axis0.pos_estimate = 0  # Reset angular position value
            odrv.axis0.requested_state = 8  # Closed-loop control
            odrv.axis0.controller.config.control_mode = 2  # Velocity Control (sharp or ramped)

            # For sharp velocity control
            #odrv.axis0.controller.config.input_mode = 1  # for PASSTHROUGH

            # For ramped velocity control
            odrv.axis0.controller.config.vel_ramp_rate = config.MOTOR_ACCELERATION  # in turns/s^2
            odrv.axis0.controller.config.input_mode = 2 # for VEL_RAMP
                        
        return odrv

    except Exception as e:
        print(f"Error: {e}")
        return None


def set_cart_velocity(odrv, state, target_velocity):
    '''
    Set the cart velocity based on the state
    :param odrv: ODrive object
    :param state: Current state of the cart
    :param target_velocity: Target velocity in turns/s
    '''
    if state == config.STATE["STOP"]:
        odrv.axis0.controller.input_vel = 0

    elif state == config.STATE["FORWARD"]:
        odrv.axis0.controller.input_vel = -target_velocity/3 # sign for logical direction of travel 

    elif state == config.STATE["BACKWARD"]:
        odrv.axis0.controller.input_vel = target_velocity

    elif state == config.STATE["TRACKING"]:
        odrv.axis0.controller.input_vel = target_velocity
    else:
        odrv.axis0.controller.input_vel = 0

def get_data(odrv):
    """
    Get data from ODrive
    :param odrv: ODrive object
    :return: Tuple of position, velocity and current
    """
    angular_position = odrv.axis0.pos_estimate  # in turns
    angular_velocity = odrv.axis0.vel_estimate  # in turns/s
    torque = odrv.axis0.motor.torque_estimate   # in Nm

    return angular_position, angular_velocity, torque
    
    