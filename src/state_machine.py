import config


def update(state, remote_command, obst):
    """
    Update the state of the cart based on the button pressed and the obstacle detected
    :param state: Current state of the cart, type STATE
    :param remote_command: Button command, type REMOTE_COMMAND
    :param obst: Obstacle detected
    :return: Updated state of the cart
    """



    if state == config.STATE["STOP"]:

        if button == config.REMOTE_COMMAND["NONE"]:
            state = config.STATE["STOP"]


        elif button == config.REMOTE_COMMAND["GO_TRACKING"]:
            state = config.STATE["TRACKING"]

    
        elif button == config.REMOTE_COMMAND["GO_BACKWARD"]:
            state = config.STATE["BACKWARD"]


        elif button == config.REMOTE_COMMAND["NOTHING"]:
            


        elif button == config.REMOTE_COMMAND["GO_FORWARD"]:


        elif button == config.REMOTE_COMMAND["GO_STOP"]:


    
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
