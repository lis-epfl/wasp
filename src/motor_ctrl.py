import numpy as np
import odrive
import time
import os

import config


def linear_to_angular(linear_value):
    """
    Convert linear value to angular value.
    :param linear_value: Linear value in meters
    :return angular_value: Angular value in turns
    """
    angular_value = linear_value / (config.PULLEY_RADIUS * 2 * np.pi)
    return angular_value


def angular_to_linear(angular_value):
    """
    Convert angular value to linear value.
    :param angular_value: Angular value in turns
    :return linear_value: Linear value in meters
    """
    linear_value = config.PULLEY_RADIUS * (2 * np.pi * angular_value)
    return linear_value


def motor_init():
    """
    Connect to ODrive and configure it
    :return odrv: ODrive object if found, None otherwise
    """
    try:
        print("Searching for ODrive...")
        odrv = odrive.find_any(timeout=20)

        if odrv is None:
            print("No ODrive found!")

        else:
            print("ODrive found!")

            odrv.clear_errors()                                                                     # Clear potential errors/disarm reason from last run
            odrv.config.dc_bus_undervoltage_trip_level = config.MIN_VOLTAGE
            odrv.config.dc_bus_overvoltage_trip_level = config.MAX_VOLTAGE
            odrv.axis0.requested_state = 8                                                          # Closed-loop control
            odrv.axis0.controller.config.control_mode = 3                                           # Position control

            odrv.axis0.controller.config.input_mode = 5                                             # Trapezoidal velocity profile
            odrv.axis0.controller.config.vel_limit = np.inf                                         # Disable maximum system velocity (does not correspond to the maximum speed of the profile)
            odrv.axis0.trap_traj.config.accel_limit = linear_to_angular(config.ACCELERATION)                        
            odrv.axis0.trap_traj.config.decel_limit = linear_to_angular(config.DECELERATION)
                                                  
            odrv.axis0.config.motor.current_soft_max = config.SOFT_MAX_CURRENT
            odrv.axis0.config.motor.current_hard_max = config.HARD_MAX_CURRENT

            odrv.axis0.controller.config.pos_gain = config.POS_GAIN                                 # Proportional gain for position loop [(rev/s) / rev]
            odrv.axis0.controller.config.vel_gain = config.VEL_GAIN                                 # Proportional gain for velocity loop  [Nm / (rev/s)]
            odrv.axis0.controller.config.vel_integrator_gain = config.INTEGRATOR_GAIN               # Integral gain for velocity loop [(Nm/s) / (rev/s)] 
        
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
    :return output: Smoothed value
    """
    a = curr_val - prev_val
    output = prev_val + a*(1.0 - np.e**(-dt*cutoff_freq))
    return output


def set_position(odrv, position):
    """
    Set the position of the motor
    :param odrv: ODrive object
    :param position: Position in meters
    """
    odrv.axis0.controller.input_pos = - linear_to_angular(position)     # in turns (minus sign because of the motor direction)


def set_velocity(odrv, velocity):
    """
    Set the velocity of the motor
    :param odrv: ODrive object
    :param velocity: Velocity in meters per second
    """
    odrv.axis0.controller.input_pos = odrv.axis0.pos_estimate           # so that the controller thinks that the trajectory is done
    odrv.axis0.trap_traj.config.vel_limit = linear_to_angular(velocity) # in turns/s


def get_data(odrv):
    """
    Get data from ODrive
    :param odrv: ODrive object
    :return: Tuple of position, velocity, torque, voltage and current 
    """
    angular_position = - odrv.axis0.pos_estimate                        # in turns    (minus sign because of the motor direction)
    angular_velocity = - odrv.axis0.vel_estimate                        # in turns/s  (minus sign because of the motor direction)
    torque = odrv.axis0.motor.torque_estimate                           # in Nm
    linear_position = angular_to_linear(angular_position)               # in m
    linear_velocity = angular_to_linear(angular_velocity)               # in m/s
    voltage = odrv.vbus_voltage                                         # in V
    current = odrv.axis0.motor.foc.Iq_measured                          # in A

    # Estimate decelerating distance relative to current velocity
    decel_dist = ((linear_velocity ** 2) / (2 * config.DECELERATION)) * config.DECELERATION_MARGIN

    return angular_position, angular_velocity, torque, linear_position, linear_velocity, voltage, current, decel_dist


def log_motor_data(timestamp, angular_position, angular_velocity, torque, linear_position, linear_velocity, tracking_error, voltage, current, x_ref, estimated_position, estimated_velocity):
    """
    Log motor data
    :param: Motor data
    :returns: Dictionary containing motor data
    """
    return {
        "Timestamp [s]": np.round(timestamp, 3),
        "Angular position [turns]": np.round(angular_position, 3),
        "Angular velocity [turns/s]": np.round(angular_velocity, 3),
        "Torque [Nm]": np.round(torque, 3),
        "Linear position [m]": np.round(linear_position, 3),
        "Linear speed [m/s]": np.round(linear_velocity, 3),
        "Voltage [V]": np.round(voltage, 3),
        "Current [A]": np.round(current, 3),
        "Reference x_ref [m]": np.round(x_ref, 3),
        "Estimated plane position [m]": np.round(estimated_position, 3),
        "Estimated plane velocity [m/s]": np.round(estimated_velocity, 3) if estimated_velocity is not None else float('nan'),
    }