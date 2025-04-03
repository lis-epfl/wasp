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
    linear_speed = config.PULLEY_RADIUS * (2 * np.pi * angular_speed)
    return linear_speed


def compute_linear_position(angular_position):
    """
    Compute linear position from angular position
    :param angular_position: Angular position in turns
    :return: Linear position in m
    """
    linear_position = config.PULLEY_RADIUS * (2 * np.pi * angular_position)
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
            odrv.config.dc_bus_undervoltage_trip_level = 12.0 # 8.0V
            odrv.config.dc_bus_overvoltage_trip_level = 17.0 # 17.0V
            
            odrv.axis0.pos_estimate = 0  # Reset angular position value
            odrv.axis0.requested_state = 8  # Closed-loop control
            odrv.axis0.controller.config.control_mode = 2  # Velocity Control (sharp or ramped)

            # For sharp velocity control
            odrv.axis0.controller.config.input_mode = 1  # for PASSTHROUGH

            # For ramped velocity control
            # odrv.axis0.controller.config.vel_ramp_rate = 5.0 # acceleration in turn/s^2
            # odrv.axis0.controller.config.input_mode = 2 # for VEL_RAMP
                        

        return odrv

    except Exception as e:
        print(f"Error: {e}")
        return None

# control motor through terminal
# def motor_control(stdscr):

#     odrv = motor_init()

#     stdscr.clear()
#     stdscr.addstr("Use arrow keys to control the motor. Press SPACE to stop. Press 'q' to exit.\n")
#     stdscr.refresh()
#     stdscr.nodelay(True)  # Make getch() non-blocking

#     while True:
#         os.system('clear')  # For Linux/macOS, use 'cls' for Windows

#         # Print motor data with formatted velocity
#         print(f"Velocity: {odrv.axis0.vel_estimate:.2f} turns/s")  # 2 decimal places
#         print(f"Angular position: {odrv.axis0.pos_estimate:.2f} turns")
#         print(f"Linear position: {compute_linear_position(odrv.axis0.pos_estimate):.2f} m")

#         #print(f"Current state: {odrv.axis0.current_state}")
#         #print(f"Velocity Limit: {odrv.axis0.controller.config.vel_limit}")
#         #print(f"Control mode: {odrv.axis0.controller.config.control_mode}")
#         print(f"Input velocity: {odrv.axis0.controller.input_vel}")
#         #print(f"Input mode: {odrv.axis0.controller.config.input_mode}")

#         # Read key (non-blocking)
#         key = stdscr.getch()
#         if key == curses.KEY_RIGHT:
#             odrv.axis0.controller.input_vel = config.MOTOR_SPEED
#         elif key == curses.KEY_LEFT:
#             odrv.axis0.controller.input_vel = -config.MOTOR_SPEED
#         elif key == ord(' '):
#             odrv.axis0.controller.input_vel = 0
#         elif key == ord('q'):
#             odrv.axis0.controller.input_vel = 0
#             break
        
#         time.sleep(0.1)  # Update every 100ms

# curses.wrapper(motor_control)


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
        odrv.axis0.controller.input_vel = target_velocity

    elif state == config.STATE["BACKWARD"]:
        odrv.axis0.controller.input_vel = -target_velocity

    elif state == config.STATE["TRACKING"]:
        odrv.axis0.controller.input_vel = target_velocity
    else:
        odrv.axis0.controller.input_vel = 0

    

    


    
