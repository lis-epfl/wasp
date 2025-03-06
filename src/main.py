import motor_control
import ultrasonic



#--------------------------------------------------------CONSTANTS--------------------------------------------------------
# General
STATE = {
    "STOP": 0,
    "FORWARD": 1,
    "BACKWARD": 2,
    "TRACKING": 3
}

# Motor
MOTOR_SPEED = 20            # in turns/sec
PULLEY_RADIUS = 0.05        # in meters (tacking into acount cable radius)

# Ultrasonic sensors
TIMEOUT1 = 1000             # in microseconds
TIMEOUT2 = 10000            # in microseconds
MAX_DIST = 6                # in meters
PIN_FRONT = 5               # GPIO pin
PIN_BACK = 16               # GPIO pin
BW_THRESHOLD = 60           # in centimeters
FW_THRESHOLD = 100          # in centimeters

# Camera


# LEDs


# RF communication



#-------------------------------------------------------------------------------------------------------------------------

def update_state(state, button, obst):
    """
    Update the state of the cart based on the button pressed and the obstacle detected
    :param state: Current state of the cart
    :param button: Button pressed
    :param obst: Obstacle detected
    :return: Updated state of the cart
    """

    if state == STATE["STOP"]:        
        if button == 1:
            state = STATE["STOP"]
        elif button == 2:
            state = STATE["FORWARD"]
        elif button == 3:
            state = STATE["TRACKING"]
        elif button == 4:
            state = STATE["BACKWARD"]
    
    elif state == STATE["FORWARD"]:
        if obst or button in [1, 3, 4]:
            state = STATE["STOP"]
        else:
            state = STATE["FORWARD"]
    
    elif state == STATE["BACKWARD"]:
        if obst or button in [1, 2, 3]:
            state = STATE["STOP"]
        else:
            state = STATE["BACKWARD"]
    
    elif state == STATE["TRACKING"]:
        if obst or button in [1, 2, 4]:
            state = STATE["STOP"]
        else:
            state = STATE["TRACKING"]
    
    return state


def main():

    # Inititialization
    motor_init()
    ultrasonic_init()
    camera_init()
    leds_init()

    state = STATE["STOP"] 

    # Main loop
    while True:
        button = get_button() # 1: Middle, 2: Up, 3: Right, 4: Down
        obst = is_there_obstacle() # True if obstacle detected, False otherwise

        state = update_state(state, button, obst)

        target_velocity = get_target_velocity() # Get UAV linear velocity from camera

        set_cart_velocity(state, target_velocity)

        time.sleep(0.1)


if __name__ == '__main__':
    main()
