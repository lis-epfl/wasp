import simple_pid as pid
import numpy as np
import odrive
import curses
import time
import os

# Set motor speed
MOTOR_SPEED = 10  # Speed in turns/sec

# Connect to ODrive
try:
    print("Searching for ODrive...")
    odrv = odrive.find_any(timeout=20)  # Timeout in seconds
    if odrv is None:
        print("No ODrive found!")
    else:
        print("ODrive found!")
        print("ODrive's configuration:")
        print(odrv)  # will print the motor's configuration
except Exception as e:
    print(f"Error: {e}")

# Ensure motor is in closed-loop control mode
odrv.axis0.requested_state = 8  # Closed-loop control
odrv.axis0.controller.config.control_mode = 2  # Velocity Control
odrv.axis0.controller.config.input_mode = 2  # Set to DIRECT_VELOCITY

def motor_control(stdscr):
    stdscr.clear()
    stdscr.addstr("Use arrow keys to control the motor. Press SPACE to stop. Press 'q' to exit.\n")
    stdscr.refresh()
    
    stdscr.nodelay(True)  # Make getch() non-blocking

    while True:
        os.system('clear')  # For Linux/macOS, use 'cls' for Windows

        # Print motor data with formatted velocity
        print(f"Velocity: {odrv.axis0.vel_estimate:.2f} turns/s")  # 2 decimal places
        print(f"Position: {odrv.axis0.pos_estimate:.2f} turns")
        print(f"Current state: {odrv.axis0.current_state}")
        print(f"Velocity Limit: {odrv.axis0.controller.config.vel_limit}")
        print(f"Control mode: {odrv.axis0.controller.config.control_mode}")
        print(f"Input velocity: {odrv.axis0.controller.input_vel}")
        print(f"Input mode: {odrv.axis0.controller.config.input_mode}")

        # Read key (non-blocking)
        key = stdscr.getch()
        if key == curses.KEY_RIGHT:
            odrv.axis0.controller.input_vel = MOTOR_SPEED
        elif key == curses.KEY_LEFT:
            odrv.axis0.controller.input_vel = -MOTOR_SPEED
        elif key == ord(' '):
            odrv.axis0.controller.input_vel = 0
        elif key == ord('q'):
            odrv.axis0.controller.input_vel = 0
            break
        
        time.sleep(0.1)  # Update every 100ms

curses.wrapper(motor_control)