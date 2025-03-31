import config


def update(last_state, remote_command, obstacle_forward, obstacle_backward):
    """
    Update the state of the cart based on the button pressed and the obstacle detected.
    :param last_state: Last state of the cart
    :param remote_command: Command received from the remote control
    :param obstacle_forward: True if obstacle detected in front, False otherwise
    :param obstacle_backward: True if obstacle detected in back, False otherwise
    :return: New state of the cart
    """
    state = last_state  # Default state to avoid uninitialized variable

    if last_state == config.STATE["STOP"]:
        if remote_command == config.REMOTE_COMMAND["GO_TRACKING"] and not (obstacle_backward or obstacle_forward):
            state = config.STATE["TRACKING"]
        elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"] and not obstacle_backward:
            state = config.STATE["BACKWARD"]
        elif remote_command == config.REMOTE_COMMAND["GO_FORWARD"] and not obstacle_forward:
            state = config.STATE["FORWARD"]
        elif remote_command == config.REMOTE_COMMAND["GO_STOP"]:
            state = config.STATE["STOP"]

    elif last_state == config.STATE["FORWARD"]:
        if obstacle_forward:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_TRACKING"]:
            state = config.STATE["TRACKING"]
        elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"]:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_FORWARD"]:
            state = config.STATE["FORWARD"]
        elif remote_command == config.REMOTE_COMMAND["GO_STOP"]:
            state = config.STATE["STOP"]

    elif last_state == config.STATE["BACKWARD"]:
        if obstacle_backward:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_TRACKING"]:
            state = config.STATE["TRACKING"]
        elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"]:
            state = config.STATE["BACKWARD"]
        elif remote_command == config.REMOTE_COMMAND["GO_FORWARD"]:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_STOP"]:
            state = config.STATE["STOP"]

    elif last_state == config.STATE["TRACKING"]:
        if obstacle_forward or obstacle_backward:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_TRACKING"]:
            state = config.STATE["TRACKING"]
        elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"]:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_FORWARD"]:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_STOP"]:
            state = config.STATE["STOP"]

    if (remote_command == config.REMOTE_COMMAND["NOTHING"]) or (remote_command == config.REMOTE_COMMAND["NONE"]):
        state = last_state

    return state
