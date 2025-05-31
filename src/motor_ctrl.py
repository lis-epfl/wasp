import simple_pid as pid
import numpy as np
import odrive
import curses
import time
import os

import config


def linear_to_angular(linear_value):
    """
    Convert linear value to angular value.
    :param linear_value: Linear value in m
    :return: Angular value in turns
    """
    angular_value = linear_value / (config.PULLEY_RADIUS * 2 * np.pi)  # radius * angle
    return angular_value


def angular_to_linear(angular_value):
    """
    Convert angular value to linear value.
    :param angular_value: Angular value in turns
    :return: Linear value in m
    """
    linear_value = config.PULLEY_RADIUS * (2 * np.pi * angular_value)  # radius * angle
    return linear_value


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
            print("Maximum achievable speed:", angular_to_linear(17*config.SPEED_CONSTANT*config.LOSS_CONSTANT)/60)

            # Motor configuration
            odrv.clear_errors() # clear potential errors/disarm reason from last run
            odrv.config.dc_bus_undervoltage_trip_level = config.MIN_VOLTAGE
            odrv.config.dc_bus_overvoltage_trip_level = config.MAX_VOLTAGE
            odrv.axis0.requested_state = 8                 # Closed-loop control
            odrv.axis0.controller.config.control_mode = 3  # 0: voltage control, 1: torque control, 2: velocity control, 3: position control

            # For position control
            odrv.axis0.controller.config.input_mode = 5  # POS_FILTER = 3, TRAP_TRAJ = 5
            odrv.axis0.controller.config.vel_limit = np.inf # to avoid maximum speed limit (does not correspond to the maximum speed of the profile)
            odrv.axis0.trap_traj.config.accel_limit = linear_to_angular(config.MAX_ACCELERATION)                        
            odrv.axis0.trap_traj.config.decel_limit = linear_to_angular(config.MAX_ACCELERATION)
            # maximum speed of the profile is set in the main loop because it depends on the mode (calibration or not)                                                  
                                                  

            odrv.axis0.config.motor.current_soft_max = config.SOFT_MAX_CURRENT
            odrv.axis0.config.motor.current_hard_max = config.HARD_MAX_CURRENT

            odrv.axis0.controller.config.pos_gain = config.POS_GAIN                    # Proportional gain for position loop [(rev/s) / rev]
            odrv.axis0.controller.config.vel_gain = config.VEL_GAIN                    # Proportional gain for velocity loop  [Nm / (rev/s)]
            odrv.axis0.controller.config.vel_integrator_gain = config.INTEGRATOR_GAIN  # Integral gain for velocity loop [(Nm/s) / (rev/s)]                        
        return odrv

    except Exception as e:
        print(f"Error: {e}")
        return None


def low_pass(curr_val, prev_val, cutoff_freq, dt):
    """
    Low pass filter to smooth the data
    :param curr_val: Current value
    :param prev_val: Previous value
    :param cutoff_freq: Cutoff frequency
    :param dt: Time step
    :return: Smoothed value
    """
    a = curr_val - prev_val # Amplitude or error
    output = prev_val + a*(1.0 - np.e**(-dt*cutoff_freq))
    return output


def set_position(odrv, position):
    """
    Set the position of the motor
    :param odrv: ODrive object
    :param position: Position in turns
    """
    odrv.axis0.controller.input_pos = - position  # in turns (minus sign because of the motor direction)


def get_data(odrv):
    """
    Get data from ODrive
    :param odrv: ODrive object
    :return: Tuple of position, velocity and current
    """
    angular_position = - odrv.axis0.pos_estimate                    # in turns    (minus sign because of the motor direction)
    angular_velocity = - odrv.axis0.vel_estimate                    # in turns/s  (minus sign because of the motor direction)
    torque = odrv.axis0.motor.torque_estimate                       # in Nm
    linear_position = angular_to_linear(angular_position)           # in m
    linear_velocity = angular_to_linear(angular_velocity)           # in m/s
    voltage = odrv.vbus_voltage                                     # in V
    current = odrv.axis0.motor.foc.Iq_measured                      # in A

    return angular_position, angular_velocity, torque, linear_position, linear_velocity, voltage, current


def log_motor_data(timestamp, angular_position, angular_velocity, torque, linear_position, linear_velocity, tracking_error, voltage, current, x_ref, estimated_position, estimated_velocity):
    return {
        "Timestamp [s]": timestamp,
        "Angular position [turns]": angular_position,
        "Angular velocity [turns/s]": angular_velocity,
        "Torque [Nm]": torque,
        "Linear position [m]": linear_position,
        "Linear speed [m/s]": linear_velocity,
        "Tracking error [m]":  tracking_error if tracking_error is not None else float('nan'),
        "Voltage [V]": voltage,
        "Current [A]": current,
        "x_ref [m]": x_ref,
        "Estimated plane plosition [m]": estimated_position,
        "Estimated plane velocity [m/s]": estimated_velocity if estimated_velocity is not None else float('nan'),
    } 