import time
import config
import leds_ctrl
import ultrasonic_ctrl
#import motor_control

def update_state(state, button, obst):
    """
    Update the state of the cart based on the button pressed and the obstacle detected
    :param state: Current state of the cart
    :param button: Button pressed
    :param obst: Obstacle detected
    :return: Updated state of the cart
    """

    if state == config.STATE["STOP"]:        
        if button == 1:
            state = config.STATE["STOP"]
        elif button == 2:
            state = config.STATE["FORWARD"]
        elif button == 3:
            state = config.STATE["TRACKING"]
        elif button == 4:
            state = config.STATE["BACKWARD"]
    
    elif state == config.STATE["FORWARD"]:
        if obst or button in [1, 3, 4]:
            state = config.STATE["STOP"]
        else:
            state = config.STATE["FORWARD"]
    
    elif state == config.STATE["BACKWARD"]:
        if obst or button in [1, 2, 3]:
            state = config.STATE["STOP"]
        else:
            state = config.STATE["BACKWARD"]
    
    elif state == config.STATE["TRACKING"]:
        if obst or button in [1, 2, 4]:
            state = config.STATE["STOP"]
        else:
            state = config.STATE["TRACKING"]
    
    return state


def main():
    # Inititialization
    leds = leds_ctrl.leds_init()
    ultrasonic_ctrl.ultrasonic_init()

    state = config.STATE["STOP"] 

    while True:

        #button = get_button() # 1: Middle, 2: Up, 3: Right, 4: Down
        #obst = is_there_obstacle() # True if obstacle detected, False otherwise

        button = 1
        obst = False
        state = update_state(state, button, obst)
        
        state = config.STATE["FORWARD"] 

        leds_ctrl.leds_set_color(leds, state)

  

        # Replace this with proper function that returns either true or false
        front_value = ultrasonic_ctrl.get_distance(config.PIN_FRONT)
        back_value = ultrasonic_ctrl.get_distance(config.PIN_BACK)

        print('Back: {:.2f} m    |    Front: {:.2f} m'.format(front_value, back_value))

        #target_velocity = get_target_velocity() # Get UAV linear velocity from camera

        #set_cart_velocity(state, target_velocity)

        time.sleep(config.DT)

if __name__ == '__main__':
    main()
