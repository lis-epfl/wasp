import simple_pid as pid
import numpy as np
import odrive
import curses
import time
import os

import config

def compute_angular_speed(linear_speed):
    """
    Compute angular speed from linear speed  
    :param linear_speed: Linear speed in m/s
    :return: angular speed in turns/s
    """
    angular_speed = linear_speed/(config.PULLEY_RADIUS * 2 * np.pi)
    return angular_speed


def compute_linear_speed(angular_speed):
    """
    Compute linear speed from angular speed
    :param angular_speed: Angular speed in turns/s
    :return: Linear speed in m/s
    """
    linear_speed = config.PULLEY_RADIUS * (2 * np.pi * angular_speed) # radius * angular speed
    return linear_speed


def compute_angular_position(linear_position):
    """
    Compute angular position from linear position
    :param linear_position: Linear position in m
    :return: Angular position in turns
    """
    angular_position = linear_position/(config.PULLEY_RADIUS * 2 * np.pi) # radius * angle
    return angular_position


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
            print("Maximum achievable speed:", compute_linear_speed(odrv.vbus_voltage*config.SPEED_CONSTANT*config.LOSS_CONSTANT)/60)

            # Motor configuration
            # dump_errors(odrv, clear=True) # Clear errors and disarm reason, needs to restart the ODrive after that
            odrv.config.dc_bus_undervoltage_trip_level = config.MIN_VOLTAGE
            odrv.config.dc_bus_overvoltage_trip_level = config.MAX_VOLTAGE
            odrv.axis0.requested_state = 8                 # Closed-loop control
            odrv.axis0.controller.config.control_mode = 3  # 0: voltage control, 1: torque control, 2: velocity control, 3: position control

            if config.CALIBRATING:
                odrv.axis0.pos_estimate = compute_angular_position(-config.INITAL_MOTOR_POS_CALIB)  
            else:
                odrv.axis0.pos_estimate = compute_angular_position(-config.INITAL_MOTOR_POS)

            # For position control
            odrv.axis0.controller.config.input_mode = 5  # POS_FILTER = 3, TRAP_TRAJ = 5
            odrv.axis0.controller.config.vel_limit = np.inf # to avoid maximum speed limit
            odrv.axis0.trap_traj.config.vel_limit = compute_angular_speed(max(config.MAX_TRACKING_SPEED, config.MAX_MANUAL_SPEED))  
            odrv.axis0.trap_traj.config.accel_limit = config.MAX_ACCELERATION                        
            odrv.axis0.trap_traj.config.decel_limit = config.MAX_ACCELERATION                                                  

            odrv.axis0.config.motor.current_soft_max = config.SOFT_MAX_CURRENT
            odrv.axis0.config.motor.current_hard_max = config.HARD_MAX_CURRENT

            odrv.axis0.controller.config.pos_gain = config.POS_GAIN                    # Proportional gain for position loop [(rev/s) / rev]
            odrv.axis0.controller.config.vel_gain = config.VEL_GAIN                    # Proportional gain for velocity loop  [Nm / (rev/s)]
            odrv.axis0.controller.config.vel_integrator_gain = config.INTEGRATOR_GAIN  # Integral gain for velocity loop [(Nm/s) / (rev/s)]                        
        return odrv

    except Exception as e:
        print(f"Error: {e}")
        return None


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
    linear_position = compute_linear_position(angular_position)     # in m
    linear_velocity = compute_linear_speed(angular_velocity)        # in m/s
    voltage = odrv.vbus_voltage                                     # in V
    current = odrv.axis0.motor.foc.Iq_measured                      # in A

    return angular_position, angular_velocity, torque, linear_position, linear_velocity, voltage, current


def log_motor_data(timestamp, angular_position, angular_velocity, torque, linear_position, linear_velocity, tracking_error, voltage, current, x_ref):
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
    } 